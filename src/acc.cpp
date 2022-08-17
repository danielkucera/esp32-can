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
  // Send CAN Message
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x368;
  tx_frame.FIR.B.DLC = 8;

  uint8_t* data = acc_message.U;

  acc_message.B.ACS_Checksum = data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7];

  for (int i=0; i<tx_frame.FIR.B.DLC; i++){
    tx_frame.data.u8[i] = data[i];
  }

  if (false) {
    for (int i=0; i<tx_frame.FIR.B.DLC; i++){
      Serial.printf("%02x", tx_frame.data.u8[i]);
    }
    Serial.printf("\n");
  }

  int ret = 1;
  while(ret){
    int ret = ESP32Can.CANWriteFrame(&tx_frame, 1000);
  }

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

void send_gra() {
  // Send CAN Message
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = GRA_NEU_ID;
  tx_frame.FIR.B.DLC = 4;

  uint8_t* data = gra_message.U;

  gra_message.B.GRA_Checksum = data[1] ^ data[2] ^ data[3];

  for (int i=0; i<tx_frame.FIR.B.DLC; i++){
    tx_frame.data.u8[i] = data[i];
  }

  if (false) {
    for (int i=0; i<tx_frame.FIR.B.DLC; i++){
      Serial.printf("%02x", tx_frame.data.u8[i]);
    }
    Serial.printf("\n");
  }

  int ret = 1;
  while(ret){
    int ret = ESP32Can.CANWriteFrame(&tx_frame, 1000);
  }
}

bool keypress_acc(char ch){
    switch (ch){
      case 'm':
        gra_message.B.GRA_Zeitluecke = 2;
        gra_message.B.GRA_Neu_Zaehler += 1;
        send_gra();
        break;
      case 'n':
        gra_message.B.GRA_Zeitluecke = 1;
        gra_message.B.GRA_Neu_Zaehler += 1;
        send_gra();
        break;
    }
}