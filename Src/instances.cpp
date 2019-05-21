//
// Created by demid on 21.05.19.
//

#include "main.h"
#include "uart_driver.h"

UartDriver uart_servo(huart3);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    uart_servo.txCompleteSync();
}