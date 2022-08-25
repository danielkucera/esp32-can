#include "main.h"
#include <ESP32CAN.h>
#include <can_regdef.h>

#define TCP_MSG_CNT 10              //number of messages to queue per write

CAN_device_t CAN_cfg;               // CAN Config

CAN_frame_t rx_frame[TCP_MSG_CNT];
CAN_frame_t tx_frame[TCP_MSG_CNT];

const int rx_queue_size = 512;       // Receive Queue size

int rx_count = 0;

//pthread_t readThread;
//pthread_t statsThread;

//unsigned char base64[(TCP_MSG_CNT*16*4)/3+3];


void can_init(){
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_23;
  CAN_cfg.rx_pin_id = GPIO_NUM_22;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));

  CAN_filter_t p_filter;
  p_filter.FM = Dual_Mode;

  p_filter.ACR0 = ((BREMSE_5_ID >> 3) & 0xff);
  p_filter.ACR1 = ((BREMSE_5_ID << 5) & 0xff);
  p_filter.ACR2 = ((GRA_NEU_ID >> 3) & 0xff);
  p_filter.ACR3 = ((GRA_NEU_ID << 5) & 0xff);

  p_filter.AMR0 = 0x00;
  p_filter.AMR1 = 0x1F;
  p_filter.AMR2 = 0x00;
  p_filter.AMR3 = 0x1F;
  ESP32Can.CANConfigFilter(&p_filter);

  ESP32Can.CANInit();
}

void report_can(){
  printf("time:%ld rcvd: %d, txerr:%d rx_overrun:%d ", millis(), rx_count, MODULE_CAN->TXERR.U, ESP32Can.CANOverrunCounter());
  rx_count = 0;
}

void can_read(){
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

      // if gra message, copy to gra
      if (rx_frame[i].MsgID == GRA_NEU_ID){
        memcpy(gra_message.U, rx_frame[i].data.u8, 4);
      }

      rx_count++;
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

#if false
    unsigned int base64_length = encode_base64((u_char*)(&rx_frame), to_send, base64+1);
    base64[base64_length+1]='\n';

    int snt = Serial.write(base64, base64_length+2);
    
    //printf("after write\n");
    if (snt < to_send){
      printf("short write %d of %d\n", snt, to_send);
    }
#endif

}

int can_send(int msg_id, uint8_t* data, int len, int timeout){
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = msg_id;
    tx_frame.FIR.B.DLC = len;

    for (int i=0; i<len; i++){
        tx_frame.data.u8[i] = data[i];
    }

#if DEBUG_TX_MSG
    for (int i=0; i<len; i++){
        Serial.printf("%02x", tx_frame.data.u8[i]);
    }
    Serial.printf("\n");
#endif

  int ret = 1;
  while(ret){
    ret = ESP32Can.CANWriteFrame(&tx_frame, 1000);
  }

  return ret;
}