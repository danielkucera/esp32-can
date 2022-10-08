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
  IrReceiver.begin(15, ENABLE_LED_FEEDBACK);
#endif

  can_init();
  epb_init();
  //acc_init();

  Serial.println("setup finished");
}

void loop() {
  static unsigned long last_report;
  can_read();

#if IR_ENABLE
  if (IrReceiver.decode()) {
    uint32_t ir_msg = IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume(); // Enable receiving of the next value

    Serial.println(ir_msg, HEX);
    keypress_acc(ir_msg);
  }
#endif

  process_epb();

  //process_acc();


  if (Serial.available() > 0) {
    int cmd = Serial.read();
    Serial.printf("received %c\n", cmd);

    keypress_epb(cmd);
    keypress_acc(cmd);
  }

  if (millis() > last_report + 1000){
    last_report = millis();
    report_can();
    report_epb();
  }

}
