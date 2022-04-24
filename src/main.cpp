#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <can_regdef.h>
#include "base64.hpp"

char incomingPacket[255];  // buffer for incoming packets

CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 512;       // Receive Queue size

pthread_t readThread;
pthread_t statsThread;
int msg_count = 0;

#define TCP_MSG_CNT 50              //number of messages to queue per write
CAN_frame_t rx_frame[TCP_MSG_CNT];
unsigned char base64[(TCP_MSG_CNT*16*4)/3+3];

void report_stats(){
  printf("loop time:%ld rcvd: %d, txerr:%d rx_overrun:%d \n", millis(), msg_count, MODULE_CAN->TXERR.U, ESP32Can.CANOverrunCounter());
  msg_count = 0;
}

void read_bus(){
  // Receive next CAN frame from queue
  unsigned long start = micros();
  int i = 0;

  //printf("bw %ld\n", micros());

  while ((i<TCP_MSG_CNT) && (micros() < start + 1000)){
    if (xQueueReceive(CAN_cfg.rx_queue, &(rx_frame[i]), 5*portTICK_PERIOD_MS) == pdTRUE) {
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
  unsigned int base64_length = encode_base64((u_char*)(&rx_frame), to_send, base64+1);
  base64[base64_length+1]='\n';

  int snt = Serial.write(base64, base64_length+2);
  
  //printf("after write\n");
  if (snt < to_send){
    printf("short write %d of %d\n", snt, to_send);
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

void setup() {
  Serial.begin(2000000);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");

  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_23;
  CAN_cfg.rx_pin_id = GPIO_NUM_22;
  Serial.println("xQueueCreate");
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module

  Serial.println("CANInit");
  ESP32Can.CANInit();

  Serial.println("pthread_create");
  pthread_create(&readThread, NULL, readFunction, NULL);
  //pthread_create(&statsThread, NULL, statsFunction, NULL);

  base64[0] = ':';

  Serial.println("setup finished");
}

void loop() {
  static unsigned long last_report;
  read_bus();

  if (millis() > last_report + 1000){
    last_report = millis();
    report_stats();
  }

}
