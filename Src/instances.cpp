//
// Created by demid on 21.05.19.
//

#include "main.h"
#include "uart_driver.h"

UartDriver uart_servo(huart3, DE_PIN_GPIO_Port, DE_PIN_Pin);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    uart_servo.txCompleteSync();
}