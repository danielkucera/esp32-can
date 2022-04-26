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

int status_sent = 0;
bool status_trigger = 0;

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

void trigger_epb_status() {
  status_trigger = 1;
}

typedef union {
	uint8_t U[8]; /**< \brief Unsigned access */
	struct {
    /* byte 0 */
		uint8_t EP1_Zaehler : 4;
		uint8_t EP1_Failure_Sta : 2;
		bool EP1_Sta_EPB : 1;
		bool EP1_Sta_Schalter : 1;
    /* byte 1 */
		uint8_t EP1_Spannkraft : 5;
		uint8_t EP1_Schalterinfo : 2;
		bool EP1_Sta_NWS : 1;
    /* byte 2 */
		uint8_t EP1_Neig_winkel : 8;
    /* byte 3 */
		uint8_t EP1_Verzoegerung : 8;
    /* byte 4 */
		bool EP1_Failureeintr : 1;
		bool EP1_Freigabe_Ver : 1;
		bool EP1_AutoHold_zul : 1;
		bool EP1_AutoHold_active : 1;
		bool EP1_SleepInd : 1;
		bool EP1_Status_Kl_15 : 1;
		bool EP1_Lampe_AutoP : 1;
		bool EP1_Bremslicht : 1;
    /* byte 5 */
		bool EP1_Warnton1 : 1;
		bool EP1_Warnton2 : 1;
		bool EP1_AnfShLock : 1;
		bool EPB_Autoholdlampe : 1;
		bool Unknown : 1;
		bool EP1_KuppModBer : 1;
    bool Unknown2 : 1;
		bool EP1_HydrHalten : 1;
    /* byte 6 */
		bool EP1_Fkt_Lampe : 1;
		bool EP1_Warnton : 1;
		bool EP1_Failure_BKL : 1;
		bool EP1_Failure_gelb : 1;
		uint8_t EP1__Text : 4;
    /* byte 7 */
		uint8_t EP1_Checksum : 8;
	} B;
} epb_message_t;

epb_message_t epb_message;

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
  tx_frame.MsgID = 0x5c0;
  tx_frame.FIR.B.DLC = 8;

  uint8_t* data = epb_message.U;

  data[7] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6];

  if (false) {
    for (int i=0; i<8; i++){
      Serial.printf("%02x", data[i]);
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

  if (status_trigger) {
    status_trigger = 0;
    send_epb_status();
  }

  if (millis() > last_report + 1000){
    last_report = millis();
    report_stats();
  }

}
