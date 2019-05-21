//
// Created by demid on 21.05.19.
//

#include "uart_driver.h"

UartDriver::UartDriver(UART_HandleTypeDef &huart, GPIO_TypeDef *dePin, uint16_t dePinNum )
    : huart_(huart), dePin_(dePin), dePinNum_(dePinNum){

}

UartDriver::~UartDriver() {}

void UartDriver::init() {
    if(!dePin_)
        return;

    sync = xSemaphoreCreateBinary();
    if(sync != NULL){
        is_inited = true;
    }
}

bool UartDriver::send(void *buf, uint16_t len, uint32_t timeout_ms) {
    if(!is_inited)
        return false;

    bool result = false;
    HAL_GPIO_WritePin(dePin_, dePinNum_, GPIO_PIN_SET);
    if(HAL_UART_Transmit_IT(&huart_, (uint8_t*)buf, len) == HAL_OK){
        if(xSemaphoreTake(sync, timeout_ms) == pdTRUE){
            result = true;
        }
    }
    HAL_GPIO_WritePin(dePin_, dePinNum_, GPIO_PIN_RESET);

    return result;
}

void UartDriver::txCompleteSync() {
    if(!is_inited)
        return;

    BaseType_t reschedule = false;
    xSemaphoreGiveFromISR(sync, &reschedule);
    if(reschedule){
        portYIELD_FROM_ISR(reschedule);
    }
}
