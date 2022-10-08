#include "main.h"

ACC_System acc_message;
GRA_Neu gra_message;

bool acc_trigger = 0;

void trigger_acc_status() {
  acc_trigger = 1;
}

void acc_init() {
  acc_message.B.ACS_Sollbeschl = 0xFE; // ? 12bits?
  acc_message.B.ACS_Sta_ADR = 2;
  acc_message.B.ACS_zul_Regelabw = 0xFE;

}

void send_acc_status() {

  uint8_t* data = acc_message.U;

  acc_message.B.ACS_Checksum = data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];

  int ret = can_send(0x368, acc_message.U, 8, 0);

  acc_message.B.ACS_Zaehler++;

}

void setup_acc_timer() {
  hw_timer_t * timer2 = NULL;

  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &trigger_acc_status, true);
  timerAlarmWrite(timer2, 45000, true); //each 45ms
  timerAlarmEnable(timer2);
}

void process_acc(){
/*
  if (acc_trigger) {
    acc_trigger = 0;
    //send_acc_status();
  }
*/
}

void report_acc() {
    printf("Down_kurz:%d Up_kurz:%d, Down_lang:%d Up_lang:%d Recall:%d Neu_Setzen:%d Zeitluecke:%d \n", 
    gra_message.B.GRA_Down_kurz,
    gra_message.B.GRA_Up_kurz,
    gra_message.B.GRA_Down_lang,
    gra_message.B.GRA_Up_lang,
    gra_message.B.GRA_Recall,
    gra_message.B.GRA_Neu_Setzen,
    gra_message.B.GRA_Zeitluecke);
}

void send_gra() {
  uint8_t* data = gra_message.U;

  gra_message.B.GRA_Neu_Zaehler++;
  gra_message.B.GRA_Checksum = data[1] ^ data[2] ^ data[3];

  int ret = 1;
  while(ret){
    int ret = can_send(GRA_NEU_ID, gra_message.U, 4, 1000);
  }
}

bool keypress_acc(uint32_t ch){
    switch (ch){
      case 'm':
      case 0xA659FF00:
        Serial.printf("DIST_MINUS\n");
        override_gra = DIST_MINUS;
        notify_gra();
        //override_times = 2;
        return 1;
        break;
      case 'n':
      case 0xA956FF00:
        Serial.printf("DIST_PLUS\n");
        override_gra = DIST_PLUS;
        notify_gra();
        //override_times = 3;
        return 1;
        break;
    }
    return 0;
}