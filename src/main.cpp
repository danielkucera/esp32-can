#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <can_regdef.h>
#include <WiFi.h>
#include <WiFiAP.h>

const char* ssid     = "ESP32-CAN";
const char* password = "123456789";

char incomingPacket[255];  // buffer for incoming packets

WiFiServer server(3333);
WiFiClient client;

CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 500;       // Receive Queue size

pthread_t readThread;
int msg_count = 0;

#define TCP_MSG_CNT 10              //number of messages to queue per write

void *readFunction(void* p){
  CAN_frame_t rx_frame[TCP_MSG_CNT];

  while (true){
    // Receive next CAN frame from queue
    for (int i=0; i<10; ){
      if (xQueueReceive(CAN_cfg.rx_queue, &(rx_frame[i]), 3 * portTICK_PERIOD_MS) == pdTRUE) {
        msg_count++;
        i++;
      }
    }

    if (client){
      client.write((u_char*)(&rx_frame), TCP_MSG_CNT*sizeof(CAN_frame_t));
    }

  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");

  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");

  IPAddress IP = WiFi.softAPIP();

  Serial.println(IP);

  server.begin();

  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_23;
  CAN_cfg.rx_pin_id = GPIO_NUM_22;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();

  pthread_create(&readThread, NULL, readFunction, NULL);
}

void loop() {

  // accept new client
  if (server.hasClient()) {
    client = server.available();
    printf("TCP client connected: %s\n", client.remoteIP().toString().c_str());
  }

  if (client){
    if (client.available()){
      int dataLen = client.read();

      byte idHigh = client.read();
      byte idLow = client.read();

      // Send CAN Message
      CAN_frame_t tx_frame;
      tx_frame.FIR.B.FF = CAN_frame_std;
      tx_frame.MsgID = idHigh*256 + idLow;
      tx_frame.FIR.B.DLC = dataLen;

      for (int i=0; i<dataLen; i++){
        tx_frame.data.u8[i] = client.read();
      }

      ESP32Can.CANWriteFrame(&tx_frame);
    }
  }

  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (lastMillis + 1000 < currentMillis) {
    lastMillis = currentMillis;
    printf("loop time:%d rcvd: %d, txerr:%d \n", (int)currentMillis, msg_count, MODULE_CAN->TXERR.U);
    msg_count = 0;
  }
}
