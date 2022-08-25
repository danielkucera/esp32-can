#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "messages.h"

#define LED 2

#define BREMSE_5_ID 0x04a8
#define GRA_NEU_ID 0x038a
#define MEPB1_ID 0x5c0

#define OFF false
#define ON true

extern Bremse_5 abs_message;
extern mEPB_1 epb_message;
extern ACC_System acc_message;
extern GRA_Neu gra_message;

void log(const char * logline);

void can_init();
void can_read();
int can_send(int msg_id, uint8_t* data, int len, int timeout);
void report_can();

void epb_init();
void process_epb();
bool keypress_epb(char ch);
void report_epb();

bool keypress_acc(char ch);

#endif