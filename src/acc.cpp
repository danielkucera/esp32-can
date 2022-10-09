#include "main.h"

ACC_System acc_message;
GRA_Neu gra_message;

enum action {
  DIST_PLUS = 1,
  DIST_MINUS,
  SPEED_PLUS,
  SPEED_MINUS,
  ACC_SET,
  ACC_RESET
};

bool acc_trigger = 0;

int override_gra = 0;
int override_times = 0;

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

  int ret = can_send(GRA_NEU_ID, gra_message.U, 4, 20);
}

void notify_gra(){
  if (override_gra) {
    switch (override_gra) {
      case DIST_MINUS:
        gra_message.B.GRA_Zeitluecke = 1;
        printf("overriding GRA ZL1\n");
        break;
      case DIST_PLUS:
        gra_message.B.GRA_Zeitluecke = 2;
        printf("overriding GRA ZL2\n");
        break;
      case SPEED_PLUS:
        gra_message.B.GRA_Up_kurz = 1;
        printf("overriding GRA UP\n");
        break;
      case SPEED_MINUS:
        gra_message.B.GRA_Down_kurz = 1;
        printf("overriding GRA DOWN\n");
        break;
      case ACC_SET:
        gra_message.B.GRA_Neu_Setzen = 1;
        printf("overriding GRA ACC SET\n");
        break;
      case ACC_RESET:
        gra_message.B.GRA_Recall = 1;
        printf("overriding GRA ACC RESET\n");
        break;
    }

    send_gra();

    override_times--;
    if (!override_times){
      override_gra = 0;
    }


    printf("overriden GRA\n");
  }

}

int set_override(int value){
  override_gra = value;
  override_times = 6;
  return 1;
}

bool keypress_acc(uint32_t ch){
    switch (ch){
      case 'm':
      case BTN_LEFT:
        printf("DIST_MINUS\n");
        return set_override(DIST_MINUS);
        break;
      case 'n':
      case BTN_RIGHT:
        printf("DIST_PLUS\n");
        return set_override(DIST_PLUS);
        break;
      case BTN_VOL_PLUS:
        printf("SPEED_PLUS\n");
        return set_override(SPEED_PLUS);
        break;
      case BTN_VOL_MINUS:
        printf("SPEED_MINUS\n");
        return set_override(SPEED_MINUS);
        break;
      case BTN_PICKUP:
        printf("ACC_SET\n");
        return set_override(ACC_SET);
        break;
      case BTN_HANGUP:
      case BTN_SEL:
        printf("ACC_RESET\n");
        return set_override(ACC_RESET);
        break;
    }
    return 0;
}