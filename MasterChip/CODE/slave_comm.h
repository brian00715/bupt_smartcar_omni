#ifndef SLAVE_COMM_H_
#define SLAVE_COMM_H_

#include "ch32v10x.h"

void SlaveComm_Init();
void SlaveComm_UARTCallback();
void SlaveComm_Exe();

extern uint8_t EncoderDataUpdated;
extern uint8_t SlaveComm_MotorSelfCheck;

#endif
