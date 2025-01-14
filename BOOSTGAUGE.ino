#include <ELMduino.h>
#include <lvgl.h>
#if LV_USE_TFT_ESPI // MUST SET IN LV_CONF
#include <TFT_eSPI.h>
#endif
//#include "TouchDrvCSTXXX.hpp"

#define TFT_HOR_RES   240
#define TFT_VER_RES   240
#define TFT_ROTATION  LV_DISPLAY_ROTATION_90
#define PIN_3V3 18
#define PIN_RX 17
#define PIN_TX 16
//CST816S mytouch(sda,scl,rst,irq);

#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];
lv_obj_t *psiScale;
lv_obj_t *needle;

#define ELM_SER Serial1
ELM327 myELM;
/*use Arduinos millis() as tick source*/
static uint32_t my_tick(void) { return millis(); }
static const float C_kPa = 17.37;

typedef struct struct_boost {
  float iatxc = 499.1 * C_kPa; //   K
  float mafxiat = iatxc * 131.9; //   gps
  float abskPa = mafxiat / 4900.25; // rpm
  int boostAngle = int(trunc(0.145038 * (abskPa - 101) * 10)); // atm in kPa
} struct_boost;

volatile struct_boost boostData;
volatile uint8_t curPID = 0;
volatile bool dataReady = false;
volatile bool queryInProgress = false;

const char* pids[] = {
  "010F", // iat
  "0110", // maf
  "010C", // rpm
  "0133"  // atm
};

void handleIAT();
void handleMAF();
void handleRPM();
void handleATM(); // void handleBAT();
void (*pidHandlers[])() = {
  handleIAT,
  handleMAF,
  handleRPM,
  handleATM
};

const int PID_COUNT = sizeof(pids)/sizeof(pids[0]);
static lv_style_t txt_style;
static lv_style_t icon_style;
lv_color_t black = LV_COLOR_MAKE(0,0,0);
lv_color_t grey = LV_COLOR_MAKE(95,95,95);
lv_color_t dark_grey = LV_COLOR_MAKE(57,57,57);
lv_color_t red = LV_COLOR_MAKE(255,19,0);

#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
    LV_UNUSED(level);
    Serial.println(buf);
    Serial.flush();
}
#endif

LV_FONT_DECLARE(microgramma);
LV_IMAGE_DECLARE(NEEDLE1);
LV_IMAGE_DECLARE(SLINE);

void setup() {
  digitalWrite(PIN_3V3,HIGH); //lv for level translator to ELM327 5V
  Serial.begin(115200);
  ELM_SER.setTX(PIN_TX);
  ELM_SER.setRX(PIN_RX);
  ELM_SER.begin(115200); // debug, timeout
  if (!myELM.begin(ELM_SER, true, 2000))  {
    Serial.println("Couldn't connect to OBD scanner");
  } else {
    Serial.println("OBD scanner connected!");
  }
  lv_init();
  lv_tick_set_cb(my_tick);
#if LV_USE_LOG != 0
  lv_log_register_print_cb( my_print );
#endif
  /*Initialize the display*/
  lv_display_t *disp;
  disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, TFT_ROTATION);
  // /*Initialize the input device driver*/
  // lv_indev_drv_t indev_drv;
  // lv_indev_drv_init(&indev_drv);             /*Descriptor of a input device driver*/
  // indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
  // indev_drv.read_cb = touchpad_read;      /*Set your driver function*/
  // lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/
  // /*Initialize the (dummy) input device driver*/. NEWER
  // lv_indev_t *indev = lv_indev_create();
  // lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
  // lv_indev_set_read_cb(indev, my_touchpad_read);
// lv_display_get_horizontal_resolution(NULL), lv_display_get_vertical_resolution(NULL));
  psiScale = lv_scale_create(lv_screen_active());
  lv_obj_set_size(psiScale, 237, 237);
  lv_scale_set_label_show(psiScale, true);
  lv_scale_set_mode(psiScale, LV_SCALE_MODE_ROUND_INNER);
  lv_obj_center(psiScale);
  lv_scale_set_total_tick_count(psiScale, 30);
  lv_scale_set_major_tick_every(psiScale, 2);
  lv_obj_set_style_length(psiScale, 5, LV_PART_ITEMS);
  lv_obj_set_style_length(psiScale, 8, LV_PART_INDICATOR);
  lv_scale_set_range(psiScale, 0, 75);
  lv_scale_set_angle_range(psiScale,150);
  lv_scale_set_rotation(psiScale,180);
  static const char *psi_custom_labels[] = {
    "          0 PSI","","2","","4","","6","","8","","10","","12","","14",NULL
  };
  lv_scale_set_text_src(psiScale, psi_custom_labels);
  lv_obj_set_style_bg_opa(psiScale, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(psiScale,lv_color_hex(0x051209),0);
  lv_obj_set_style_radius(psiScale,240,0);
  lv_obj_set_style_outline_color(psiScale,lv_palette_darken(LV_PALETTE_GREY,2),0);
  needle = lv_image_create(lv_screen_active());
  lv_image_set_src(needle,&NEEDLE1);
  lv_scale_set_image_needle_value(psiScale,needle,164);


  lv_obj_t *barScale = lv_scale_create(lv_screen_active());
  lv_obj_set_size(barScale, 219, 219);
  // lv_scale_set_label_show(barScale, true);
  lv_scale_set_mode(barScale, LV_SCALE_MODE_ROUND_OUTER);
  lv_obj_center(barScale);
  lv_scale_set_total_tick_count(barScale, 5);
  lv_scale_set_major_tick_every(barScale, 6);
  lv_obj_set_style_length(barScale, 5, LV_PART_ITEMS);
  lv_obj_set_style_length(barScale, 8, LV_PART_INDICATOR);
  lv_scale_set_range(barScale, 0, 20);
  lv_scale_set_angle_range(barScale,40);
  lv_scale_set_rotation(barScale,140);
  static const char *bar_custom_labels[] = { "                            -1 BAR\n", NULL };
  lv_scale_set_text_src(barScale, bar_custom_labels);
 // lv_obj_t *bar_needle = lv_image_create(lv_screen_active(), NULL);


  static lv_style_t psi_indicator_style;
  lv_style_init(&psi_indicator_style);
  /* Label style properties */
  lv_style_set_text_font(&psi_indicator_style, &microgramma);
  lv_style_set_text_color(&psi_indicator_style, lv_palette_lighten(LV_PALETTE_GREY, 1));
  /* Major tick properties */
  lv_style_set_line_color(&psi_indicator_style, lv_palette_darken(LV_PALETTE_GREY, 1));
  lv_style_set_width(&psi_indicator_style, 13U);      /*Tick length*/
  lv_style_set_line_width(&psi_indicator_style, 2U);  /*Tick width*/
  lv_obj_add_style(psiScale, &psi_indicator_style, LV_PART_INDICATOR);

  static lv_style_t bar_indicator_style;
  lv_style_init(&bar_indicator_style);
  /* Label style properties */
  lv_style_set_text_font(&bar_indicator_style, &lv_font_montserrat_10);
  lv_style_set_text_color(&bar_indicator_style, lv_palette_lighten(LV_PALETTE_GREY, 1));
  /* Major tick properties */
  lv_style_set_line_color(&bar_indicator_style, lv_palette_darken(LV_PALETTE_GREY, 1));
  lv_style_set_width(&bar_indicator_style, 9U);      /*Tick length*/
  lv_style_set_line_width(&bar_indicator_style, 2U);  /*Tick width*/
  lv_obj_add_style(barScale, &bar_indicator_style, LV_PART_INDICATOR);

  static lv_style_t minor_ticks_style;
  lv_style_init(&minor_ticks_style);
  lv_style_set_line_color(&minor_ticks_style, lv_palette_darken(LV_PALETTE_GREY, 1));
  lv_style_set_width(&minor_ticks_style, 4U);         /*Tick length*/
  lv_style_set_line_width(&minor_ticks_style, 2U);    /*Tick width*/
  lv_obj_add_style(psiScale, &minor_ticks_style, LV_PART_ITEMS);
  lv_obj_add_style(barScale, &minor_ticks_style, LV_PART_ITEMS);

  static lv_style_t main_line_style;
  lv_style_init(&main_line_style);
  /* Main line properties */
  lv_style_set_arc_color(&main_line_style, lv_palette_darken(LV_PALETTE_GREY, 1));
  lv_style_set_arc_width(&main_line_style, 2U); /*Tick width*/
 // lv_obj_add_style(psiScale, &main_line_style, LV_PART_MAIN);
  lv_obj_add_style(barScale, &main_line_style, LV_PART_MAIN);
  /* Add a section */
  static lv_style_t section_minor_tick_style;
  static lv_style_t section_label_style;
  static lv_style_t section_main_line_style;

  lv_style_init(&section_label_style);
  lv_style_init(&section_minor_tick_style);
  lv_style_init(&section_main_line_style);

  /* Label style properties */
  lv_style_set_text_font(&section_label_style, &microgramma);
  lv_style_set_text_color(&section_label_style, lv_palette_darken(LV_PALETTE_RED, 2));

  lv_style_set_line_color(&section_label_style, lv_palette_darken(LV_PALETTE_RED, 2));
  lv_style_set_line_width(&section_label_style, 2U); /*Tick width*/

  lv_style_set_line_color(&section_minor_tick_style, lv_palette_darken(LV_PALETTE_RED, 2));
  lv_style_set_line_width(&section_minor_tick_style, 2U); /*Tick width*/

  /* Main line properties */
  lv_style_set_arc_color(&section_main_line_style, lv_palette_darken(LV_PALETTE_RED, 2));
  lv_style_set_arc_width(&section_main_line_style, 4U); /*Tick width*/

  /* Configure section styles */
  lv_scale_section_t *section = lv_scale_add_section(psiScale);
  lv_scale_section_set_range(section, 50, 75);
  lv_scale_section_set_style(section, LV_PART_INDICATOR, &section_label_style);
  lv_scale_section_set_style(section, LV_PART_ITEMS, &section_minor_tick_style);
  lv_scale_section_set_style(section, LV_PART_MAIN, &section_main_line_style);

  lv_obj_set_style_bg_color(lv_screen_active(), lv_palette_darken(LV_PALETTE_BLUE,4),LV_PART_MAIN);
  lv_obj_t *sline = lv_image_create(lv_screen_active());
  lv_image_set_src(sline,&SLINE);
  lv_obj_center(sline);

}

void loop() {
  if (!dataReady && myELM.nb_rx_state != ELM_GETTING_MSG) {
    myELM.sendCommand(pids[curPID]);
  } else {
    noInterrupts();
    dataReady = false;
    interrupts();
    pidHandlers[curPID]();
    if (PID_COUNT == curPID - 1) {
      lv_scale_set_image_needle_value(psiScale,needle,boostData.boostAngle);
    }
    curPID = curPID++ % PID_COUNT;
  }
  lv_timer_handler();
  delay(5); // TODO REMOVE
}

void serialEvent1() {
  if (ELM_SER.available()) { dataReady = true; }
}

void handleIAT() {
  float temp = myELM.intakeAirTemp();
  if (ELM_SUCCESS != myELM.nb_rx_state) {
    Serial.println("Failed IAT read!");
  } else {
    boostData.iatxc = (temp + 273.15f) * C_kPa;
  }
}

void handleMAF() {
  float maf = myELM.mafRate();
  if (ELM_SUCCESS != myELM.nb_rx_state) {
    Serial.println("Failed MAF read!");
  } else {
    boostData.mafxiat = boostData.iatxc * maf;
  }

}
void handleRPM() {
  float rpm = myELM.rpm();
  if (ELM_SUCCESS != myELM.nb_rx_state) {
    Serial.println("Failed RPM read!");
  } else {
    boostData.abskPa = boostData.mafxiat / rpm;
  }
}
void handleATM() {
  int atm = int(myELM.absBaroPressure());
  float intermediate;
  if (ELM_SUCCESS != myELM.nb_rx_state) {
    Serial.println("Failed ATM read!");
  } else {
    if (atm < boostData.abskPa) {
      intermediate = 0.145038 * (boostData.abskPa - atm);
    } else {
      intermediate = -((atm - boostData.abskPa) * 50); // 2 / 100); 100kPa/bar, bar scale double
    }
    boostData.boostAngle = int(trunc(intermediate * 10)) + 180;
  }
}

// void my_disp_flush(lv_display_t *disp, const lv_area_t *area, lv_color_t *color_p) {
//   uint16_t c;

//   tft.startWrite(); /* Start new TFT transaction */
//   tft.setAddrWindow(area->x1, area->y1, (area->x2 - area->x1 + 1), (area->y2 - area->y1 + 1)); /* set the working window */
//   for (int y = area->y1; y <= area->y2; y++) {
//     for (int x = area->x1; x <= area->x2; x++) {
//       c = color_p->full;
//       tft.writeColor(c, 1);
//       color_p++;
//     }
//   }
//   tft.endWrite(); /* terminate TFT transaction */
//   lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
// }

// void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data) { // bool?
//   last_x = 0;
//   last_y = 0;
//   if(mytouch.available()) {
//     last_x = mytouch.data.x;
//     last_y = mytouch.data.y;
//     data->state = LV_INDEV_STATE_PR;
//   } else {
//     data->state = LV_INDEV_STATE_REL;
//   }
//   data->point.x = last_x;
//   data->point.y = last_y;
// }
// void lv_port_indev_init(void) {
//   static lv_indev_drv_t indev_drv;
//   /*Register a touchpad input device*/
//   lv_indev_drv_init(&indev_drv);
//   indev_drv.type = LV_INDEV_TYPE_POINTER;
//   indev_drv.read_cb = touchpad_read;
//   indev_touchpad = lv_indev_drv_register(&indev_drv);
// } //https://forum.lvgl.io/t/how-to-use-cst816s-with-lvgl-for-gestures/11528