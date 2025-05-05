#include <ELMduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

#define ARDUINO_ARCH_RP2040 1 // import proper TFT_eSPI settings
#define PSI_CONV_FACTOR 0.145038f // PSI/kPa
#define PSI_SCALE       10
#define PIN_3V3      18
#define PIN_RX       17
#define PIN_TX       16
#define TFT_HOR_RES  240
#define TFT_VER_RES  TFT_HOR_RES
#define TFT_ROTATION LV_DISPLAY_ROTATION_90
#define ELM_TIMEOUT  1000

#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES * 2 / 3)
uint32_t draw_buf1[DRAW_BUF_SIZE / 4];
uint32_t draw_buf2[DRAW_BUF_SIZE / 4];
lv_obj_t *psi_scale;
lv_obj_t *bar_scale;
lv_obj_t *needle;
lv_display_t *disp;

#define ELM_SER   Serial1
#define ELM_DEBUG false
ELM327 my_ELM;

static const float C_kPa = 17.37; // rev/(min*°K*s^2) * 1kPa/1000Pa

enum State { IDLE, WAITING };
State OBDstate = IDLE;

typedef struct struct_boost { // IAT * MAF * C / RPM
  float iat = 225.9; //   °K
  float rpm = 3900.25; // rev/min TODO 1min/60s???
  float maf = 131.9; //   g/s
  uint8_t atm = 101; //   kPa
  bool is_psi = true;
  uint8_t boost_angle = 133;
} struct_boost;

volatile struct_boost boost_data;
volatile uint8_t cur_PID = 0;
volatile uint8_t try_again = 0;
volatile unsigned long last_time;
volatile unsigned long atm_time; // check once per minute, maybe 10s
volatile uint32_t increment = 0;
const int PID_COUNT = 4;
  //                               iat   rpm   maf   atm
const uint8_t PIDs[PID_COUNT] = { 0x0F, 0x0C, 0x10, 0x33 };
const char compPIDs[PID_COUNT] = { 'F', 'C', '0', '3' };

lv_color_t grey = LV_COLOR_MAKE(95,95,95);
lv_color_t red = LV_COLOR_MAKE(255,19,0);
lv_color_t main_color;
lv_color_t blue;

LV_FONT_DECLARE(microgramma);
LV_IMAGE_DECLARE(NEEDLE1);
LV_IMAGE_DECLARE(SLINE);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_3V3,OUTPUT);
  digitalWrite(PIN_3V3,HIGH); //lv for level translator to ELM327 5V
  ELM_SER.setTX(PIN_TX);
  ELM_SER.setRX(PIN_RX);
  ELM_SER.begin(38400);
  if (!ELM_SER) { while(1); } // do something here if serial port didn't start
  if (!my_ELM.begin(ELM_SER, ELM_DEBUG, ELM_TIMEOUT)) { try_again = 5; }

  main_color = lv_palette_lighten(LV_PALETTE_GREY, 1);
  blue = lv_palette_darken(LV_PALETTE_BLUE,4);
  make_screen_elements();
  make_styles();

  if (try_again > 0) {
    if (!my_ELM.begin(ELM_SER, ELM_DEBUG, ELM_TIMEOUT)) {
      if (0 == try_again) {
        while(1); // No ELM available after multiple tries
      } else { --try_again; }
      delay(100);
    } else { try_again = 0; }
  }
  last_time = millis();
  atm_time = millis();
}

void loop() {
  if (millis() - last_time >= increment || increment > 1000) {
    increment = lv_timer_handler();
    last_time = millis();
  }
  switch (OBDstate) {
    case IDLE:
      my_ELM.queryPID(1, PIDs[cur_PID], 1);
      OBDstate = WAITING;
      break;
    case WAITING:
      int8_t readState = my_ELM.get_response();
      if (ELM_GETTING_MSG != readState && ELM_TIMEOUT != readState) {
        if (compPIDs[cur_PID] == my_ELM.payload[3]) {
          uint64_t data = my_ELM.findResponse();
          switch (PIDs[cur_PID]) {
            case 0x0F: //iat in Celcius
              boost_data.iat = data + 233.15f; // + 273.15 (C to K) - 40 (IAT bias)
              break;
            case 0x0C: //rpm x 4
              boost_data.rpm = data / 4.0f;
              break;
            case 0x10: //maf (g/s) x 100
              boost_data.maf = data / 100.0f;
              break;
            case 0x33: //atm (kPa)
              boost_data.atm = (uint8_t)data;
          }
          if (cur_PID < 2) {
            cur_PID = ++cur_PID;
          } else if (millis() > atm_time) {
            cur_PID = 3;
            atm_time = millis() + 10000;
          } else {
            cur_PID = 0;
          }
        }
        OBDstate = IDLE;
        my_ELM.flushInputBuff();
        if (0 == cur_PID) {
          calc_angle();
          lv_tick_inc(millis() - last_time);
        }
      }
  }
}

void calc_angle() {
  float abs_kPa = C_kPa * boost_data.iat * boost_data.maf / boost_data.rpm;
  float intermediate;
  boost_data.is_psi = boost_data.atm <= abs_kPa ? true : false;
  Serial.println(abs_kPa - boost_data.atm); // send pressure data to pi for alt display and graphing
  if (boost_data.is_psi) { intermediate = PSI_CONV_FACTOR * (abs_kPa - boost_data.atm); }
  else { intermediate = max(4.0 - ((float(boost_data.atm) - abs_kPa) / 25), 0.0); }// 100kPa/bar, 4x & inverted
  boost_data.boost_angle = uint8_t(trunc(intermediate * PSI_SCALE));
  lv_scale_set_image_needle_value((boost_data.is_psi ? psi_scale : bar_scale), needle, boost_data.boost_angle);
}

typedef struct {
    TFT_eSPI *tft;
} lv_tft_espi_t;

lv_display_t *my_tft_espi_create(uint8_t hor_res, uint8_t ver_res, void *buf1, void *buf2, uint32_t buf_size) {
  lv_tft_espi_t *dsc = (lv_tft_espi_t *)lv_malloc_zeroed(sizeof(lv_tft_espi_t));
  lv_display_t *disp = lv_display_create(hor_res, ver_res);
  dsc->tft = new TFT_eSPI(hor_res, ver_res);
  dsc->tft->begin();
  dsc->tft->setRotation(TFT_ROTATION);
  dsc->tft->initDMA();
  lv_display_set_driver_data(disp, (void *)dsc);
  lv_display_set_flush_cb(disp, flush_cb);
  lv_display_set_buffers(disp, (void *)buf1, (void *)buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
  return disp;
}

static void flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map) {
  lv_tft_espi_t * dsc = (lv_tft_espi_t *)lv_display_get_driver_data(disp);
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  dsc->tft->startWrite();
  dsc->tft->setAddrWindow(area->x1, area->y1, w, h);
  dsc->tft->dmaWait();
  dsc->tft->pushPixelsDMA((uint16_t *)px_map, w * h); // byte order was reversed here, now lv_conf
  dsc->tft->endWrite();
  lv_display_flush_ready(disp);
}

void make_scale(lv_obj_t *scale, uint8_t size, bool inner, bool center, uint8_t ticks, 
    uint8_t major_every, uint16_t sweep_angle, uint16_t rot_angle, const char *labels[]) {
  lv_obj_set_size(scale, size, size);
  lv_scale_set_mode(scale, (inner ? LV_SCALE_MODE_ROUND_INNER : LV_SCALE_MODE_ROUND_OUTER));
  if (center) { lv_obj_center(scale); }
  lv_scale_set_total_tick_count(scale, ticks);
  lv_scale_set_major_tick_every(scale, major_every);
  lv_scale_set_range(scale, 0, sweep_angle);
  lv_scale_set_angle_range(scale, sweep_angle);
  lv_scale_set_rotation(scale, rot_angle);
  lv_scale_set_text_src(scale, labels);
}

void make_screen_elements() {
  lv_init();
  disp = my_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf1, draw_buf2, sizeof(draw_buf1));

  psi_scale = lv_scale_create(lv_screen_active());
  static const char *psi_custom_labels[] = {
      "         0 PSI","","2","","4","","6","","8","","10","","","13","",NULL };
  make_scale(psi_scale, 237, true, true, 31, 2, 150, 180, psi_custom_labels);
  lv_obj_set_style_bg_opa(psi_scale, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(psi_scale,lv_color_hex(0x051209),0);
  lv_obj_set_style_radius(psi_scale,240,0);
  needle = lv_image_create(lv_screen_active());
  lv_image_set_src(needle,&NEEDLE1);
  lv_scale_set_image_needle_value(psi_scale,needle,boost_data.boost_angle);

  static const char *bar_custom_labels[] = { "                           -1 BAR\n", NULL };
  bar_scale = lv_scale_create(lv_screen_active());
  make_scale(bar_scale, 219, false, true, 6, 7, 40, 140, bar_custom_labels);

  lv_obj_set_style_bg_color(lv_screen_active(), main_color, 0);
  lv_obj_t *sline = lv_image_create(lv_screen_active());
  lv_image_set_src(sline,&SLINE);
  lv_obj_center(sline);
}

void make_styles() {
  static lv_style_t psi_indicator_style;
  lv_style_init(&psi_indicator_style);
  lv_style_set_text_font(&psi_indicator_style, &microgramma);
  lv_style_set_text_color(&psi_indicator_style, main_color);
  lv_style_set_line_color(&psi_indicator_style, main_color);
  lv_style_set_length(&psi_indicator_style, 9);
  lv_style_set_line_width(&psi_indicator_style, 3);
  lv_obj_add_style(psi_scale, &psi_indicator_style, LV_PART_INDICATOR);

  static lv_style_t bar_indicator_style;
  lv_style_init(&bar_indicator_style);
  lv_style_set_text_font(&bar_indicator_style, &lv_font_montserrat_10);
  lv_style_set_text_color(&bar_indicator_style, main_color);
  lv_style_set_line_color(&bar_indicator_style, main_color);
  lv_style_set_length(&bar_indicator_style, 9);
  lv_style_set_line_width(&bar_indicator_style, 2);
  lv_obj_add_style(bar_scale, &bar_indicator_style, LV_PART_INDICATOR);

  static lv_style_t minor_ticks_style;
  lv_style_init(&minor_ticks_style);
  lv_style_set_line_color(&minor_ticks_style, main_color);
  lv_style_set_length(&minor_ticks_style, 5);
  lv_style_set_line_width(&minor_ticks_style, 2);
  lv_obj_add_style(psi_scale, &minor_ticks_style, LV_PART_ITEMS);
  lv_obj_add_style(bar_scale, &minor_ticks_style, LV_PART_ITEMS);

  static lv_style_t main_line_style;
  lv_style_init(&main_line_style);
  lv_style_set_arc_color(&main_line_style, main_color);
  lv_style_set_arc_width(&main_line_style, 2);
  lv_obj_add_style(bar_scale, &main_line_style, 0);

  static lv_style_t section_minor_tick_style;
  static lv_style_t section_label_style;
  static lv_style_t section_main_line_style;
  lv_style_init(&section_label_style);
  lv_style_init(&section_minor_tick_style);
  lv_style_init(&section_main_line_style);

  lv_style_set_text_font(&section_label_style, &microgramma);
  lv_style_set_text_color(&section_label_style, red);
  lv_style_set_line_color(&section_label_style, red);
  lv_style_set_line_width(&section_label_style, 2);
  lv_style_set_line_color(&section_minor_tick_style, red);
  lv_style_set_line_width(&section_minor_tick_style, 2);
  lv_style_set_arc_color(&section_main_line_style, red); // lv_style_set_arc_image_src(
  lv_style_set_arc_width(&section_main_line_style, 4);

  lv_scale_section_t *section = lv_scale_add_section(psi_scale);
  lv_scale_section_set_range(section, 100, 150);
  lv_scale_section_set_style(section, LV_PART_INDICATOR, &section_label_style);
  lv_scale_section_set_style(section, LV_PART_ITEMS, &section_minor_tick_style);
  lv_scale_section_set_style(section, LV_PART_MAIN, &section_main_line_style);
}