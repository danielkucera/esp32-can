#include "main.h"
#include "base64.hpp"

void log(const char * logline) {
  Serial.printf("%ld %s", millis(), logline);
}

void setup() {
  Serial.begin(460800);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0);

  can_init();
  epb_init();
  //acc_init();

  Serial.println("setup finished");
}

void loop() {
  static unsigned long last_report;
  can_read();

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
