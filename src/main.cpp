#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <can_regdef.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WebServer.h>

const char* ssid     = "ESP32-CAN";
const char* password = "123456789";

char * udpAddress = "192.168.4.2";
int udpPort = 3333;
char incomingPacket[255];  // buffer for incoming packets

WebServer server(80);

WiFiUDP udp;

CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");

  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");

  IPAddress IP = WiFi.softAPIP();

  Serial.println(IP);

  udp.begin(IP,udpPort);

  //CAN_cfg.speed = CAN_SPEED_125KBPS;
  CAN_cfg.speed = CAN_SPEED_100KBPS;
  //CAN_cfg.speed = (CAN_speed_t)62;
  CAN_cfg.tx_pin_id = GPIO_NUM_23;
  CAN_cfg.rx_pin_id = GPIO_NUM_22;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
}

unsigned long lastMillis = 0;
int doSend = 0;

void loop() {

  CAN_frame_t rx_frame;

  unsigned long currentMillis = millis();

  // Receive next CAN frame from queue
  while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.FIR.B.FF == CAN_frame_std) {
      printf("s");
    }
    else {
      printf("e");
    }

    if (rx_frame.FIR.B.RTR == CAN_RTR) {
      printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    }
    else {
      udp.beginPacket(udpAddress,udpPort);
      udp.write(rx_frame.MsgID/256);
      udp.write(rx_frame.MsgID%256);
      printf("0x%08X ", rx_frame.MsgID);
      for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
        printf("%02X", rx_frame.data.u8[i]);
        udp.write(rx_frame.data.u8[i]);
      }
      printf("\n");
      udp.endPacket();
    }
  }

  int packetSize = udp.parsePacket();
  if (packetSize){
    udpPort = udp.remotePort();
    int len = udp.read(incomingPacket, packetSize);
    if (len > 0)
    {
      printf("sending\n");

      // Send CAN Message
      CAN_frame_t tx_frame;
      tx_frame.FIR.B.FF = CAN_frame_std;
      tx_frame.MsgID = incomingPacket[0]*256 + incomingPacket[1];
      tx_frame.FIR.B.DLC = len-2;

      for (int i=0; i<len-2; i++){
        tx_frame.data.u8[i] = incomingPacket[i+2];
      }

      ESP32Can.CANWriteFrame(&tx_frame);
    }
  }

  if (lastMillis + 1000 < currentMillis) {
    lastMillis = currentMillis;
    printf("loop time:%d txerr:%d \n", (int)currentMillis, MODULE_CAN->TXERR.U);
    doSend = 0;
  }
}
