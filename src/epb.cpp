#include "main.h"
#include <Preferences.h>

#define AUTOHOLD_RESET_MS ((180-30)*1000) // time in ms after autohold start to start warning
#define REQUIRED_PEDAL_PRESSURE 100 // pedal pressure required for autohold reset
#define RESET_MINIMAL_MS (3*1000) // time in ms after autohold start during which autohold reset is disabled

Preferences preferences;

Bremse_5 abs_message;
mEPB_1 epb_message;

bool epb_trigger = 0;
int epb_sent = 0;

void trigger_epb_status() {
  epb_trigger = 1;
}

void setup_epb_timer() {
  hw_timer_t * timer = NULL;

  timer = timerBegin(0, 80, true);	
  timerAttachInterrupt(timer, &trigger_epb_status, true);
  timerAlarmWrite(timer, 20000, true); //each 20ms	
  timerAlarmEnable(timer);
}

void epb_init() {
  preferences.begin("epb", false);

  //0xa6 = 166; (166÷256)×(7,968+4,224)−7,968 = -0.06225
  epb_message.B.EP1_Verzoegerung = 0xa6;

  // should enable autohold setting in cluster
  // requires coding in gateway (enable epb)
  epb_message.B.EP1_AutoHold_zul = 1;
  epb_message.B.EP1_AutoHold_active = preferences.getBool("default-enabled", true);

  //data[4] = 0x21; b0010 0001
  //epb_message.B.EP1_Failureeintr = 1;
  //epb_message.B.EP1_Status_Kl_15 = 1;

  //data[6] = 0x08;
  //epb_message.B.EP1_Fkt_Lampe = 1;

  //*****************

  //epb_message.B.EP1_AutoHold_active = 1;
  //epb_message.B.EPB_Autoholdlampe = 1;
  //epb_message.B.EP1_AutoHold_zul = 1;

  if (true) {
    for (int i=0; i<8; i++){
      Serial.printf("%02x", epb_message.U[i]);
    }
    Serial.printf("\n");
  }

    setup_epb_timer();
}

void report_epb() {
    printf("epb_sent:%d br_light:%d br_press:%d, br_sta_dru:%d br_dru_val:%d autoh_act:%d autoh_sta:%d epb_sta:%d \n", 
    epb_sent,
    abs_message.B.BR5_Bremslicht, 
    abs_message.B.BR5_Bremsdruck,
    abs_message.B.BR5_Sta_Druck,
    abs_message.B.BR5_Druckvalid,
    abs_message.B.ESP_Autohold_active,
    abs_message.B.ESP_Autohold_Standby,
    epb_message.B.EP1_Sta_EPB);
    epb_sent = 0;
}


bool send_epb_status() {
  uint8_t* data = epb_message.U;

  data[7] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];

  int ret = can_send(MEPB1_ID, epb_message.U, 8, 3000);

  if (ret){
    printf("send epb result %d\n", ret);
    return false;
  }

  epb_message.B.EP1_Zaehler++;
  epb_sent++;
  return true;
}

void set_warning(bool status){
  static bool last_status = false;

  if (last_status != status) {
    if (last_status == OFF){
      log("!!! HOLD BRAKES !!!\n");
      epb_message.B.EP1_AnfShLock = 1; // press pedal icon
      epb_message.B.EP1__Text = 4; // EP1__Text, Text_4 "0100 Press the brake pedal"
      epb_message.B.EP1_Failure_gelb = 1;
      digitalWrite(LED, 1);
      // TODO: add beeper
    } else {
      log("!!! RELEASE BRAKES !!!\n");
      epb_message.B.EP1_AnfShLock = 0;
      epb_message.B.EP1__Text = 0;
      digitalWrite(LED, 0);
    }
  }
  last_status = status;
}

void set_epb(bool status){
  epb_message.B.EP1_Sta_EPB = status;
  set_warning(status); // EPB ON = WARNING ON
}


void process_epb(){
  if (epb_trigger) {
    epb_trigger = 0;
    send_epb_status();
  }

  static unsigned long last_autohold_activated = 0;
  static unsigned long last_autohold_deactivated = 0;
  static bool last_autohold_status = 0;
  static uint8_t last_request_status = 0;

  /*
  if (epb_message.B.EP1_AutoHold_active != ?cluster value?) {
    preferences.putBool("default-enabled", value);
    epb_message.B.EP1_AutoHold_active = value;
  }
  */

  if (last_autohold_status != abs_message.B.ESP_Autohold_active) {
    if (last_autohold_status == 0) {
      log("AUTOHOLD ACTIVATED, RELEASE BRAKES\n");
      last_autohold_activated = millis();
      set_epb(OFF);
    } else {
      log("AUTOHOLD DEACTIVATED\n");
      last_autohold_deactivated = millis();
      set_epb(OFF);
    }

  }
  last_autohold_status = abs_message.B.ESP_Autohold_active;

  if (last_request_status != abs_message.B.ESP_Anforderung_EPB) {
    if (last_request_status == 0) {
      log("ABS requesting EPB, EPB engaging\n");
      set_epb(ON);
    } else {
      //If we do engage and disengage fast enough, the pressure in the valve will not have time to release?
      log("ABS EPB request off, EPB off\n");
      set_epb(OFF);
    }
  }
  last_request_status = abs_message.B.ESP_Anforderung_EPB;

  if (abs_message.B.ESP_Autohold_active) {
    if (millis() < last_autohold_activated + RESET_MINIMAL_MS) {
      // nothing to do autohold active only very short
    } else if (abs_message.B.BR5_Bremsdruck > REQUIRED_PEDAL_PRESSURE) {
      // breake pedal pressed, reset autohold
      log("Reactivating autohold due to brake press\n");
      set_epb(ON);

    } else if (millis() > last_autohold_activated + AUTOHOLD_RESET_MS) {
      // autohold limit approaching
      set_warning(ON);

    }
  }

}

bool keypress_epb(uint32_t ch){
  switch (ch){
    case 'd':
      epb_message.B.EP1_Freigabe_Ver = !epb_message.B.EP1_Freigabe_Ver;
      Serial.printf("EP1_Freigabe_Ver = %d\n", epb_message.B.EP1_Freigabe_Ver);
      break;
    case 'w':
      epb_message.B.EP1_Verzoegerung += 1;
      Serial.printf("EP1_Verzoegerung = %x\n", epb_message.B.EP1_Verzoegerung);
      break;
    case 's':
      epb_message.B.EP1_Verzoegerung -= 1;
      Serial.printf("EP1_Verzoegerung = %x\n", epb_message.B.EP1_Verzoegerung);
      break;
  }
  return 0;
}