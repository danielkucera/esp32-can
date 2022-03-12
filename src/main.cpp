#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <can_regdef.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiManager.h>

const char* ssid     = "ESP32-CAN";
const char* password = "123456789";

WiFiManager manager; 

char incomingPacket[255];  // buffer for incoming packets

WiFiServer server(3333);
WiFiClient client;

CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 256;       // Receive Queue size

pthread_t readThread;
pthread_t statsThread;
int msg_count = 0;

#define TCP_MSG_CNT 256              //number of messages to queue per write
CAN_frame_t rx_frame[TCP_MSG_CNT];

void *readFunction(void* p){
  int i;
  unsigned long start;

  while (true){
    // Receive next CAN frame from queue
    start = micros();
    i = 0;

      printf("bw %ld\n", micros());

    while ((i<TCP_MSG_CNT) && (micros() < start + 1000)){
      if (xQueueReceive(CAN_cfg.rx_queue, &(rx_frame[i]), 5*portTICK_PERIOD_MS) == pdTRUE) {
        msg_count++;
        i++;
      } else {
        printf("queue empty\n");
      }
    }
      printf("aw %d\n", micros());

    if (i<1){
      printf("queue timeout\n");
      continue;
    }

    //Serial.printf("queue %d\n", i);

    int to_send = i*sizeof(CAN_frame_t);

    if (client){
      printf("before write %d\n", to_send);
      int snt = client.write((u_char*)(&rx_frame), to_send);
      printf("after write\n");
      if (snt < to_send){
        printf("short write %d of %d\n", snt, to_send);
      }
    } 

  }
}

void *statsFunction(void* p){
  while(true){
    printf("loop time:%ld rcvd: %d, txerr:%d \n", millis(), msg_count, MODULE_CAN->TXERR.U);
    msg_count = 0;
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");

  bool success = manager.autoConnect(ssid, password);

  if(!success) {
      Serial.println("Failed to connect");
  } 
  else {
      Serial.println("Connected");
  }

  IPAddress IP = WiFi.softAPIP();

  Serial.println(IP);

  server.begin();

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
  pthread_create(&statsThread, NULL, statsFunction, NULL);

  Serial.println("setup finished");
}

void loop() {

  // accept new client
  if (server.hasClient()) {
    client = server.available();
    printf("TCP client connected: %s\n", client.remoteIP().toString().c_str());
  }

  if (client){
    if (client.available()){
      u_char dataLen = client.peek();
      u_char buf[11];

      //printf("client data len %d\n", dataLen);

      if (dataLen > 8){
        printf("failed peek\n");
        return;
      }

      unsigned long clientStart = millis();
      while (client.available() < dataLen + 3){
        if (millis() > clientStart + 1000) {
          printf("client data wait timeout\n");
          return;
        }
      }

      int pos = client.read(buf, dataLen + 3);
      if (pos != dataLen + 3) {
          printf("client data error req %d got %d\n", dataLen + 3, pos);
          return;
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
        int ret = ESP32Can.CANWriteFrame(&tx_frame);
        if (!ret){
          break;
        }
        printf("CAN write retry %d\n", ret);
      }
      //printf("msg sent\n");
    }
  }

}
