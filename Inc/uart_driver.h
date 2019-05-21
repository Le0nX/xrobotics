//
// Created by demid on 21.05.19.
//

#ifndef STM32F103_EXPL_UART_DRIVER_H
#define STM32F103_EXPL_UART_DRIVER_H

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include ""

class UartDriver{
public:
    UartDriver(UART_HandleTypeDef &huart, GPIO_TypeDef *dePin, uint16_t dePinNum);
    ~UartDriver();
    void init();
    void txCompleteSync();
    bool send(void *buf, uint16_t len, uint32_t timeout_ms);
private:
    bool is_inited = false;
    xSemaphoreHandle sync;
    UART_HandleTypeDef &huart_;
    GPIO_TypeDef *dePin_;
    uint16_t dePinNum_;

};




#endif //STM32F103_EXPL_UART_DRIVER_H


