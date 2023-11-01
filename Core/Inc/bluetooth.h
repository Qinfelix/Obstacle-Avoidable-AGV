#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "main.h"
#include "motor.h"

void send_current_speed(MOTOR* motor,UART_HandleTypeDef* huart);
int16_t get_sign(uint8_t flag, uint8_t data);

#endif
