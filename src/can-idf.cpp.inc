#include "main.h"
#include "driver/twai.h"

#define TCP_MSG_CNT 10              //number of messages to queue per write

#pragma message "including IDF CAN driver"

const int rx_queue_size = 512;       // Receive Queue size

int rx_count = 0;

//pthread_t readThread;
//pthread_t statsThread;

//unsigned char base64[(TCP_MSG_CNT*16*4)/3+3];


void can_init(){
#if CAN_ENABLE

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  twai_filter_config_t f_config = {
    .acceptance_code = (((uint32_t)BREMSE_5_ID << 21) | (uint32_t)(GRA_NEU_ID << 5)),
    .acceptance_mask = 0x001F001F,
    .single_filter = false
  };

  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  //Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      printf("Driver installed\n");
  } else {
      printf("Failed to install driver\n");
      return;
  }

  //Start TWAI driver
  if (twai_start() == ESP_OK) {
      printf("Driver started\n");
  } else {
      printf("Failed to start driver\n");
      return;
  }

#endif
}

void report_can(){
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  printf("time:%ld rcvd: %d, txerr:%d rx_overrun:%d ", millis(), rx_count, twai_status.tx_error_counter, twai_status.rx_overrun_count);
  rx_count = 0;
}

void can_read(){
  // Receive next CAN frame from queue
  unsigned long start = micros();
  int i = 0;

#if CAN_ENABLE
  //printf("bw %ld\n", micros());
  twai_message_t message;

  while (twai_receive(&message, 0) == ESP_OK) {

    // if abs message, copy to status
    if (message.identifier == BREMSE_5_ID){
      memcpy(abs_message.U, message.data, 8);
    }

    // if gra message, copy to gra
    if (message.identifier == GRA_NEU_ID){
      memcpy(gra_message.U, message.data, 4);
      notify_gra();
    }

#if DEBUG_RX_MSG
    printf("can rcvd %x: ", message.identifier);
    for (int i=0; i<message.data_length_code; i++){
        printf("%02x", message.data[i]);
    }
    printf("\n");
#endif

    rx_count++;
    i++;
    
  }

#endif

#if DUMP_TRAFFIC
    unsigned int base64_length = encode_base64((u_char*)(&rx_frame), to_send, base64+1);
    base64[base64_length+1]='\n';

    int snt = Serial.write(base64, base64_length+2);
    
    //printf("after write\n");
    if (snt < to_send){
      printf("short write %d of %d\n", snt, to_send);
    }
#endif

}

int can_send(int msg_id, uint8_t* data, int len, unsigned long timeout){
  twai_message_t message;

  memset(&message, 0, sizeof(message));

  message.extd = 0;
  message.identifier = msg_id;
  message.data_length_code = len;
  message.rtr = 0;
  //message
  

  for (int i=0; i<len; i++){
      message.data[i] = data[i];
  }

    int ret = 0;

#if CAN_ENABLE
    //ret = 1;
    long start_ms = millis();

    esp_err_t err = ESP_FAIL;

    //while ((err != ESP_OK) && (millis() < start_ms + timeout)){
      err = twai_transmit(&message, pdMS_TO_TICKS(timeout));
    //}

    if (err != ESP_OK){
      printf("tx_err: %s (%d)\n",esp_err_to_name(err), err);
      ret = 1;
    }
#endif

#if DEBUG_TX_MSG
    printf("can send %x: ", message.identifier);
    for (int i=0; i<len; i++){
        printf("%02x", data[i]);
    }
    printf("\n");

  printf("tx ret: %d\n", err);
#endif

  return ret;
}