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

#ifndef CAN_ENABLE
#define CAN_ENABLE 1
#endif

#define DEBUG_TX_MSG 0
#define DEBUG_RX_MSG 0

#ifndef IR_ENABLE
#define IR_ENABLE 0
#endif

extern Bremse_5 abs_message;
extern mEPB_1 epb_message;
extern ACC_System acc_message;
extern GRA_Neu gra_message;

void log(const char * logline);

void can_init();
void can_read();
int can_send(int msg_id, uint8_t* data, int len, unsigned long timeout);
void report_can();

void epb_init();
void process_epb();
bool keypress_epb(uint32_t ch);
void report_epb();

bool keypress_acc(uint32_t ch);
void notify_gra();

#endif