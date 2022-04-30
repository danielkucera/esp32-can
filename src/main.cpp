#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <can_regdef.h>
#include "base64.hpp"
#include "messages.h"

#define LED 2
#define AUTOHOLD_RESET_SECONDS 10
#define BREMSE_5_ID 0x04a8
#define MEPB1_ID 0x5c0

char incomingPacket[255];  // buffer for incoming packets

CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 512;       // Receive Queue size

pthread_t readThread;
pthread_t statsThread;
int msg_count = 0;

int status_sent = 0;
bool status_trigger = 0;
bool nmh_epb_trigger = 0;

mEPB_1 epb_message;
Bremse_5 abs_message;

#define TCP_MSG_CNT 10              //number of messages to queue per write
CAN_frame_t rx_frame[TCP_MSG_CNT];
CAN_frame_t tx_frame[TCP_MSG_CNT];
unsigned char base64[(TCP_MSG_CNT*16*4)/3+3];

void report_stats(){
  printf("loop time:%ld rcvd: %d, txerr:%d rx_overrun:%d status_sent:%d \n", millis(), msg_count, MODULE_CAN->TXERR.U, ESP32Can.CANOverrunCounter(), status_sent);
  msg_count = 0;
}

void read_bus(){
  // Receive next CAN frame from queue
  unsigned long start = micros();
  int i = 0;

  //printf("bw %ld\n", micros());

  while ((i<TCP_MSG_CNT) && (micros() < start + 1000)){
    if (xQueueReceive(CAN_cfg.rx_queue, &(rx_frame[i]), 5*portTICK_PERIOD_MS) == pdTRUE) {

      // if abs message, copy to status
      if (rx_frame[i].MsgID == BREMSE_5_ID){
        memcpy(abs_message.U, rx_frame[i].data.u8, 8);
      }

      msg_count++;
      i++;
    } else {
      //printf("queue empty\n");
    }
  }
  //printf("aw %d\n", micros());

  if (i<1){
    //printf("queue timeout\n");
    return;
  }

  //Serial.printf("queue %d\n", i);

  int to_send = i*sizeof(CAN_frame_t);

  //printf("before write %d\n", to_send);

  if (false) {
    unsigned int base64_length = encode_base64((u_char*)(&rx_frame), to_send, base64+1);
    base64[base64_length+1]='\n';

    int snt = Serial.write(base64, base64_length+2);
    
    //printf("after write\n");
    if (snt < to_send){
      printf("short write %d of %d\n", snt, to_send);
    }

  }

}

void read_serial(){
  if (Serial.available()>0){
    u_char dataLen = Serial.peek();
    u_char buf[11];

    //printf("client data len %d\n", dataLen);

    if (dataLen > 8){
      printf("failed peek\n");
      //Serial.readBytes(buf, 11);
      return;
    }

    unsigned long clientStart = millis();
    while (Serial.available() < dataLen + 3){
      if (millis() > clientStart + 1000) {
        printf("client data wait timeout\n");
        return;
      }
    }

    int pos = 0;
    while ((pos += Serial.readBytes(buf+pos, dataLen - pos + 3)) < dataLen + 3) {
      printf("client read retry req %d got %d\n", dataLen + 3, pos);
    }

    // Send CAN Message
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = buf[1]*256 + buf[2];
    tx_frame.FIR.B.DLC = dataLen;

    for (int i=0; i<dataLen; i++){
      tx_frame.data.u8[i] = buf[i+3];
    }

    while (true) {
      int ret = ESP32Can.CANWriteFrame(&tx_frame, 1000000);
      if (!ret){
        break;
      }
      printf("CAN write retry %d\n", ret);
    }
    //printf("msg sent\n");
  } else {
    delay(1);
  }

}

void *readFunction(void* p){
  while (true){
    read_serial();

  }
}

void *statsFunction(void* p){
  while(true){
    report_stats();
    delay(1000);
  }
}

void trigger_epb_status() {
  status_trigger = 1;
}

void trigger_nmh_epb() {
  nmh_epb_trigger = 1;
}

void epb_init() {
  //0xa6 = 166; (166÷256)×(7,968+4,224)−7,968 = -0.06225
  epb_message.B.EP1_Verzoegerung = 0xa6;

  //data[4] = 0x21; b0010 0001
  //epb_message.B.EP1_Failureeintr = 1;
  //epb_message.B.EP1_Status_Kl_15 = 1;

  //data[6] = 0x08;
  //epb_message.B.EP1_Fkt_Lampe = 1;

  //*****************

  epb_message.B.EP1_AutoHold_active = 1;
  epb_message.B.EPB_Autoholdlampe = 1;
  epb_message.B.EP1_AutoHold_zul = 1;

  if (true) {
    for (int i=0; i<8; i++){
      Serial.printf("%02x", epb_message.U[i]);
    }
    Serial.printf("\n");
  }

}

void send_epb_status() {
  // Send CAN Message
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = MEPB1_ID;
  tx_frame.FIR.B.DLC = 8;

  uint8_t* data = epb_message.U;

  data[7] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];

  for (int i=0; i<tx_frame.FIR.B.DLC; i++){
    tx_frame.data.u8[i] = data[i];
  }

  if (false) {
    for (int i=0; i<tx_frame.FIR.B.DLC; i++){
      Serial.printf("%02x", tx_frame.data.u8[i]);
    }
    Serial.printf("\n");
  }

  int ret = ESP32Can.CANWriteFrame(&tx_frame, 1000);

  epb_message.B.EP1_Zaehler++;
  status_sent++;

}

void send_nmh_epb() {
  // Send CAN Message
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x739;
  tx_frame.FIR.B.DLC = 7;

  tx_frame.data.u8[0] = 0x04;
  tx_frame.data.u8[1] = 0x03;
  tx_frame.data.u8[2] = 0x01;

  if (false) {
    for (int i=0; i<tx_frame.FIR.B.DLC; i++){
      Serial.printf("%02x", tx_frame.data.u8[i]);
    }
    Serial.printf("\n");
  }

  int ret = ESP32Can.CANWriteFrame(&tx_frame, 1000);

  epb_message.B.EP1_Zaehler++;
  status_sent++;

}

void setup_epd_timer() {
  hw_timer_t * timer = NULL;

  timer = timerBegin(0, 80, true);	
  timerAttachInterrupt(timer, &trigger_epb_status, true);
  timerAlarmWrite(timer, 20000, true); //each 20ms	
  timerAlarmEnable(timer);

  hw_timer_t * timer2 = NULL;

  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &trigger_epb_status, true);
  timerAlarmWrite(timer2, 200000, true); //each 200ms
  timerAlarmEnable(timer2);
}

void process_input(){
  static unsigned long last_autohold_activated = 0;
  static bool last_autohold_status = 0;
  static bool autohold_reset_needed = 0;

  if (last_autohold_status != abs_message.B.ESP_Autohold_active) {
    if (last_autohold_status == 0) {
      last_autohold_activated = millis();
    }
  }
  last_autohold_status = abs_message.B.ESP_Autohold_active;

  if ((millis() > last_autohold_activated + AUTOHOLD_RESET_SECONDS*1000) && abs_message.B.ESP_Autohold_active) {
    if (!autohold_reset_needed) {
      autohold_reset_needed = 1;
      Serial.print("!!! PRESS BRAKES !!!\n");
    }
  }

  if (autohold_reset_needed) {
    epb_message.B.EP1_Warnton = 1;
    epb_message.B.EP1__Text = 4; // EP1__Text, Text_4 "0100 Press the brake pedal"

    if (abs_message.B.BR5_Bremslicht) {
      epb_message.B.EP1_Sta_EPB = 1;
    }

    if (!abs_message.B.ESP_Autohold_active) {
      epb_message.B.EP1_Sta_EPB = 0;
      autohold_reset_needed = 0;
      Serial.print("!!! AUTOHOLD REACTIVATED, RELEASE BRAKES !!!\n");
    }

  } else {
    epb_message.B.EP1_Warnton = 0;
    epb_message.B.EP1_Warnton1 = 0;
    epb_message.B.EP1_Warnton2 = 0;
    epb_message.B.EP1__Text = 0;
  }

}

void process_input_old() {
  static bool last_epb_active = 0;
  static unsigned long active_since = 0;

  digitalWrite(LED, epb_message.B.EP1_Sta_EPB);

  if (epb_message.B.EP1_Sta_EPB != last_epb_active) {
    if (!last_epb_active) { // save when we activated
      active_since = millis();
    }

  }
  last_epb_active = epb_message.B.EP1_Sta_EPB;

  if (millis() > active_since + 1000) {

    epb_message.B.EP1_Sta_EPB = 0;

  }

  if (abs_message.B.ESP_Anforderung_EPB == 2) {

    if(false){
      epb_message.B.EP1_Sta_EPB = 1;
      epb_message.B.EP1_Fkt_Lampe = 1;
    } else {
      epb_message.B.EP1_AutoHold_active = 1;
    }
  }

}

void setup() {
  Serial.begin(460800);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");

  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0);

  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_23;
  CAN_cfg.rx_pin_id = GPIO_NUM_22;
  Serial.println("xQueueCreate");
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module

  CAN_filter_t p_filter;
  p_filter.FM = Single_Mode;

  p_filter.ACR0 = ((BREMSE_5_ID >> 3) & 0xff);
  p_filter.ACR1 = ((BREMSE_5_ID << 5) & 0xff);
  p_filter.ACR2 = 0;
  p_filter.ACR3 = 0;

  p_filter.AMR0 = 0x00;
  p_filter.AMR1 = 0x1F;
  p_filter.AMR2 = 0xFF;
  p_filter.AMR3 = 0xFF;
  ESP32Can.CANConfigFilter(&p_filter);

  Serial.println("CANInit");
  ESP32Can.CANInit();

  Serial.println("pthread_create");
  //pthread_create(&readThread, NULL, readFunction, NULL);
  //pthread_create(&statsThread, NULL, statsFunction, NULL);

  base64[0] = ':';

  epb_init();
  setup_epd_timer();

  Serial.println("setup finished");
}

void loop() {
  static unsigned long last_report;
  read_bus();

  process_input();

  if (status_trigger) {
    status_trigger = 0;
    send_epb_status();
  }

  if (nmh_epb_trigger) {
    nmh_epb_trigger = 0;
    //send_nmh_epb();
  }

  if (millis() > last_report + 1000){
    last_report = millis();
    report_stats();
  }

}
