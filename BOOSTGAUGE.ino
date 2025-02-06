#include <ELMduino.h>
#include <lvgl.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include "TouchDrvCSTXXX.hpp"
#include "SensorQMI8658.hpp"

#define ARDUINO_ARCH_RP2040 1 // import proper TFT_eSPI settings
#define PSI_CONV_FACTOR 0.145038f // PSI/kPa
#define PSI_SCALE    10
#define PIN_3V3      18
#define PIN_RX       17
#define PIN_TX       16
#define TP_SDA       6
#define TP_SCL       7
#define TP_RST       22
#define TP_IRQ       21
#define CST816S_ADDR 0x15
#define IMU_IRQ      23
#define TFT_HOR_RES  240
#define TFT_VER_RES  TFT_HOR_RES
#define TFT_ROTATION LV_DISPLAY_ROTATION_90

TouchDrvCSTXXX touch;
int16_t x[5], y[5];
bool is_pressed = false;

SensorQMI8658 qmi; //imu 
volatile bool interruptFlag = false; //imu_flag
void setFlag(void) { interruptFlag = true; }

#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];
lv_obj_t *psi_scale;
lv_obj_t *bar_scale;
lv_obj_t *needle;
lv_display_t *disp;

#define ELM_SER   Serial1
#define ELM_DEBUG true
ELM327 my_ELM;
volatile bool data_ready = false;
void serialEvent1() { if (ELM_SER.available()) { data_ready = true; } }
static const float C_kPa = 17.37; // rev/m*°K*s^2

typedef struct struct_boost {
  float iat_x_c = 225.9 * C_kPa; //       °K (sec/min factored in C_kPa)
  float iat_o_rpm = iat_x_c / 3900.25; // °K/rpm
  float abs_kPa = iat_o_rpm * 131.9; //   °K*gps/rpm 
  int boost_angle = int(trunc(0.145038 * (abs_kPa - 101) * 10)); // atm in kPa
} struct_boost;

volatile struct_boost boost_data;
volatile uint8_t cur_PID = 0;
volatile uint8_t try_again = 0;
volatile bool query_in_progress = false;
lv_color_t main_color;
volatile unsigned long last_time;
volatile unsigned long try_time;
volatile uint32_t increment = 0;

const char* pids[] = {
  "010F", // iat
  "010C", // rpm
  "0110", // maf
  "0133"  // atm
};

void handle_IAT();
void handle_MAF();
void handle_RPM();
void handle_ATM(); // void handle_BAT();
void (*pid_handlers[])() = { handle_IAT, handle_RPM, handle_MAF, handle_ATM };

const int PID_COUNT = sizeof(pids)/sizeof(pids[0]);
lv_color_t grey = LV_COLOR_MAKE(95,95,95);
lv_color_t dark_grey = LV_COLOR_MAKE(57,57,57);
lv_color_t red = LV_COLOR_MAKE(255,19,0);
lv_color_t blue;

LV_FONT_DECLARE(microgramma);
LV_IMAGE_DECLARE(NEEDLE1);
LV_IMAGE_DECLARE(SLINE);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_3V3,OUTPUT);
  digitalWrite(PIN_3V3,HIGH); //lv for level translator to ELM327 5V
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, LOW);
  delay(30);
  digitalWrite(TP_RST, HIGH);
  delay(50);
  Wire1.setSDA(TP_SDA);
  Wire1.setSCL(TP_SCL);
  Wire1.begin();
  touch.setPins(TP_RST, TP_IRQ);
  if (touch.begin(Wire1, CST816S_ADDR, TP_SDA, TP_SCL)) {
    attachInterrupt(TP_IRQ, []() { is_pressed = true; }, FALLING);
  }
  qmi.setPins(IMU_IRQ);
  if (!qmi.begin(Wire1, QMI8658_L_SLAVE_ADDRESS, TP_SDA, TP_SCL)) { while(1); } // NO SENSOR!!!
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_500Hz);
  qmi.enableAccelerometer();
  uint8_t modeCtrl = SensorQMI8658::ANY_MOTION_EN_X |
                     SensorQMI8658::ANY_MOTION_EN_Y |
                     SensorQMI8658::ANY_MOTION_EN_Z |
                     SensorQMI8658::NO_MOTION_EN_X |
                     SensorQMI8658::NO_MOTION_EN_Y |
                     SensorQMI8658::NO_MOTION_EN_Z ;
  qmi.configMotion(modeCtrl, 1.0, 1.0, 1.0, 1, 0.1, 0.1, 0.1, 1, 1, 1);
  qmi.enableMotionDetect(SensorQMI8658::INTERRUPT_PIN_1);
  attachInterrupt(IMU_IRQ, setFlag, CHANGE);
  ELM_SER.setTX(PIN_TX);
  ELM_SER.setRX(PIN_RX);
  ELM_SER.begin(38400);
  if (!my_ELM.begin(ELM_SER, ELM_DEBUG, 2000)) { try_again = 5; } // NO ELM327!!!

  lv_init();
  disp = my_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
  lv_indev_t *indev = lv_indev_create(); // touch input device
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touchpad_read);
  psi_scale = lv_scale_create(lv_screen_active());
  static const char *psi_custom_labels[] = {
      "         0 PSI","","2","","4","","6","","8","","10","","","13","",NULL };
  make_scale(psi_scale, 237, true, true, 31, 2, 150, 180, psi_custom_labels);
  lv_obj_set_style_bg_opa(psi_scale, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(psi_scale,lv_color_hex(0x051209),0);
  lv_obj_set_style_radius(psi_scale,240,0);
  needle = lv_image_create(lv_screen_active());
  lv_image_set_src(needle,&NEEDLE1);
  lv_scale_set_image_needle_value(psi_scale,needle,133);

  static const char *bar_custom_labels[] = { "                           -1 BAR\n", NULL };
  bar_scale = lv_scale_create(lv_screen_active());
  make_scale(bar_scale, 219, false, true, 6, 7, 40, 140, bar_custom_labels);

  main_color = lv_palette_lighten(LV_PALETTE_GREY, 1);
  blue = lv_palette_darken(LV_PALETTE_BLUE,4);

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

  lv_obj_set_style_bg_color(lv_screen_active(), main_color, 0);
  lv_obj_t *sline = lv_image_create(lv_screen_active());
  lv_image_set_src(sline,&SLINE);
  lv_obj_center(sline);
  last_time = millis();
  try_time = millis();
}

void loop() {
  if (millis() - last_time >= increment) { // non-blocking way to periodically perform tasks
    increment = lv_timer_handler();
    last_time = millis();
  }
  if (interruptFlag) {
    noInterrupts();
    interruptFlag = false;
    interrupts();
    uint8_t status = qmi.getStatusRegister();
    if (status & SensorQMI8658::EVENT_ANY_MOTION) {} //capture millis() here
  } // does this fire too on SIGNIFICANT?
  if (try_again > 0 && millis() - try_time > 10000) {
    try_time = millis();
    if (!my_ELM.begin(ELM_SER, ELM_DEBUG, 2000)) { --try_again; }
    else { try_again = 0; }
  } else {
  // if (!data_ready) {
  //   if (my_ELM.nb_rx_state != ELM_GETTING_MSG) { my_ELM.sendCommand(pids[cur_PID]); }
  // } else {
  //   noInterrupts();
  //   data_ready = false;
  //   interrupts();
    pid_handlers[cur_PID]();
    if (PID_COUNT == cur_PID + 1) {
      int angle = boost_data.boost_angle;
      lv_scale_set_image_needle_value((angle < 0 ? bar_scale : psi_scale), needle, angle);
    }
    cur_PID = ++cur_PID % PID_COUNT;
    lv_tick_inc(millis() - last_time);
  }
}

bool check_data(String st) {
  if (ELM_SUCCESS == my_ELM.nb_rx_state) { return true; }
  else { boost_data.boost_angle = 133; return false; Serial.print("bad "); Serial.println(st);}
}

void handle_IAT() { float temp = my_ELM.intakeAirTemp();
  if (check_data("iat")) { boost_data.iat_x_c = (temp + 273.15f) * C_kPa; }
}

void handle_RPM() { float rpm = my_ELM.rpm();
  if (check_data("rpm")) { boost_data.iat_o_rpm = boost_data.iat_x_c / rpm; }
}

void handle_MAF() { float maf = my_ELM.mafRate();
  if (check_data("maf")) { boost_data.abs_kPa = boost_data.iat_o_rpm * maf; }
}

void handle_ATM() {
  uint8_t atm = int(my_ELM.absBaroPressure());
  float intermediate;
  if (check_data("atm")) {
    if (atm < boost_data.abs_kPa) { intermediate = PSI_CONV_FACTOR * (boost_data.abs_kPa - atm);
    } else { intermediate = max(40 - ((atm - boost_data.abs_kPa) / 25), 0); } 
    boost_data.boost_angle = int(trunc(intermediate * PSI_SCALE)); //\ 100kPa/bar, 4x & inverted
  }
}

typedef struct {
    TFT_eSPI *tft;
} lv_tft_espi_t;

lv_display_t *my_tft_espi_create(uint8_t hor_res, uint8_t ver_res, void *buf, uint32_t buf_size) {
  lv_tft_espi_t *dsc = (lv_tft_espi_t *)lv_malloc_zeroed(sizeof(lv_tft_espi_t));
  lv_display_t *disp = lv_display_create(hor_res, ver_res);
  dsc->tft = new TFT_eSPI(hor_res, ver_res);
  dsc->tft->begin();
  dsc->tft->setRotation(TFT_ROTATION);
  dsc->tft->initDMA();
  lv_display_set_driver_data(disp, (void *)dsc);
  lv_display_set_flush_cb(disp, flush_cb);
  lv_display_set_buffers(disp, (void *)buf, NULL, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
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

void touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  uint8_t last_x = 0;
  uint8_t last_y = 0;
  if(is_pressed) {
    is_pressed = false;
    uint8_t touched = touch.getPoint(x, y, touch.getSupportTouchPoint());
    if(touched) {
      last_x = x[0];
      last_y = y[0];
      data->state = LV_INDEV_STATE_PR;
      lv_obj_set_style_bg_color(lv_screen_active(),red,LV_PART_MAIN);
    }
  } else {
    data->state = LV_INDEV_STATE_REL;
    lv_obj_set_style_bg_color(lv_screen_active(),blue,LV_PART_MAIN);
  }
  data->point.x = last_x;
  data->point.y = last_y;
}