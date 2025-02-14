#include <ELMduino.h>

#define PSI_CONV_FACTOR 0.145038f // PSI/kPa
#define PSI_SCALE       10
#define PIN_3V3         18
#define PIN_RX          17
#define PIN_TX          16
#define ELM_SER         Serial1
#define ELM_DEBUG       true

ELM327 my_ELM;
//volatile bool data_ready = false;
//void serialEvent1() { if (ELM_SER.available()) { data_ready = true; } }
static const float C_kPa = 17.37; // rev/m*°K*s^2

typedef struct struct_boost { // IAT * MAF * C / RPM
  float iat = 225.9; //   °K
  float rpm = 3900.25; // rev/min
  float maf = 131.9; //   g/s
  uint8_t atm = 101; //   kPa
  bool is_psi = true;
  uint8_t boost_angle = 133;
} struct_boost;

volatile struct_boost boost_data;
volatile uint8_t cur_PID = 0;
volatile uint8_t try_again = 0;
volatile bool query_in_progress = false;
volatile unsigned long last_time;
volatile unsigned long try_time;

const char* pids[] = {
  "010F", // iat
  "010C", // rpm
  "0110", // maf
  "0133"  // atm
};

bool handle_IAT();
bool handle_MAF();
bool handle_RPM();
bool handle_ATM(); // void handle_BAT();
bool (*pid_handlers[])() = { handle_IAT, handle_RPM, handle_MAF, handle_ATM };

const uint8_t PID_COUNT = sizeof(pids)/sizeof(pids[0]);
volatile uint8_t state;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_3V3,OUTPUT);
  digitalWrite(PIN_3V3,HIGH); //lv for level translator to ELM327 5V
  ELM_SER.setTX(PIN_TX);
  ELM_SER.setRX(PIN_RX);
  ELM_SER.begin(38400);
  if (!my_ELM.begin(ELM_SER, ELM_DEBUG, 2000)) { try_again = 5; } // NO ELM327!!!
  last_time = millis();
  try_time = millis();
  state = my_ELM.nb_rx_state;
}

void loop() {
  uint8_t last_state = state;
  state = my_ELM.nb_rx_state;
  if (state != last_state) {
    Serial.println(last_state);
  } else { Serial.print(last_state); }
  if (try_again > 0) {
    if (millis() - try_time > 10000) {
      try_time = millis();
      if (!my_ELM.begin(ELM_SER, ELM_DEBUG, 2000)) { --try_again; }
      else { try_again = 0; }
    }
  } else {
    if (pid_handlers[cur_PID]()) {
      boost_data.boost_angle = calc_angle();
      Serial.print("angle: ");Serial.print(boost_data.is_psi ? "" : "-");Serial.println(boost_data.boost_angle);
      cur_PID = ++cur_PID % PID_COUNT;
    }
  }
}

bool handle_IAT() {
  float temp = my_ELM.intakeAirTemp();
  bool ready = 0.0 != temp;
  if (ready) { boost_data.iat = temp + 273.15f; }
  return ready;
}

bool handle_RPM() {
  float rpm = my_ELM.rpm();
  bool ready = 0.0 != rpm;
  if (ready) { boost_data.rpm = rpm; }
  return ready;
}

bool handle_MAF() {
  float maf = my_ELM.mafRate();
  bool ready = 0.0 != maf;
  if (ready) { boost_data.maf = maf; }
  return ready;
}

bool handle_ATM() {
  uint8_t atm = my_ELM.absBaroPressure();
  bool ready = 0 < atm;
  if (ready) { boost_data.atm = atm; }
  return ready;
}

uint8_t calc_angle() { // SET A FLAG FOR WHICH SCALE TO USE!!!
  float abs_kPa = C_kPa * boost_data.iat * boost_data.maf / boost_data.rpm;
  float intermediate;
  if (boost_data.atm < abs_kPa) {
    intermediate = PSI_CONV_FACTOR * (abs_kPa - boost_data.atm);
    boost_data.is_psi = true;
  } else {
    intermediate = max(4.0 - ((boost_data.atm - abs_kPa) / 25), 0.0);
    boost_data.is_psi = false;
  } 
  boost_data.boost_angle = int(trunc(intermediate * PSI_SCALE)); //\ 100kPa/bar, 4x & inverted
  return boost_data.boost_angle;
}
