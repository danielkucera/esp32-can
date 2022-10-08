#include "main.h"
#include "base64.hpp"
#include <esp_task_wdt.h>

#if IR_ENABLE
#include <IRremote.hpp>
#endif

#define D8 GPIO_NUM_8

void log(const char * logline) {
  Serial.printf("%ld %s", millis(), logline);
}

void setup() {
  Serial.begin(460800);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0);

  /// configure drive mode for transciever
  pinMode(D8, OUTPUT);
  digitalWrite(D8, 0);

#if IR_ENABLE
  //IR
  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
#endif

#ifdef IR_VCC
  pinMode(IR_VCC, OUTPUT);
  digitalWrite(IR_VCC, 1);
#endif

#ifdef IR_GND
  pinMode(IR_GND, OUTPUT);
  digitalWrite(IR_GND, 0);
#endif

  can_init();
  epb_init();
  //acc_init();

  printf("setup finished\n");
}

void loop() {
  static unsigned long last_report;
  can_read();

#if IR_ENABLE
  if (IrReceiver.decode()) {
    uint32_t ir_msg = IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume(); // Enable receiving of the next value

    printf("ir_rcvd 0x%x\n", ir_msg);
    keypress_acc(ir_msg);
  }
#endif

  process_epb();

  //process_acc();


  if (Serial.available() > 0) {
    int cmd = Serial.read();
    printf("received %c\n", cmd);

    keypress_epb(cmd);
    keypress_acc(cmd);
  }

  if (millis() > last_report + 1000){
    last_report = millis();
    report_can();
    report_epb();
  }

}
