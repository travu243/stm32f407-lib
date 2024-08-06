

/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: August 4,2024
 *      Author: tra vu
 */


#ifndef STM32F407XX_RCC_DRIVER_H_
#define STM32F407XX_RCC_DRIVER_H_



#include "stm32f407xx.h"



void RCC_SetAHBPrescaller(uint8_t AHBPrescaller);
void RCC_SetAPB1Prescaller(uint8_t APB1Prescaller);
void RCC_SetAPB2Prescaller(uint8_t APB2Prescaller);

void RCC_SelectSystemClockSource(uint8_t SysClkSrc);
void RCC_AHB1PeriphClockEnable(uint8_t PerNum);
void RCC_AHB2PeriphClockEnable(uint8_t PerNum);
void RCC_APB1PeriphClockEnable(uint8_t PerNum);
void RCC_APB2PeriphClockEnable(uint8_t PerNum);
void RCC_AHB1PeriphClockDisable(uint8_t PerNum);
void RCC_AHB2PeriphClockDisable(uint8_t PerNum);
void RCC_APB1PeriphClockDisable(uint8_t PerNum);
void RCC_APB2PeriphClockDisable(uint8_t PerNum);
void RCC_AHB1PeriphReset(uint8_t PerNum);
void RCC_APB2PeriphClockDisable(uint8_t PerNum);
void RCC_AHB2PeriphReset(uint8_t PerNum);
void RCC_APB1PeriphReset(uint8_t PerNum);
void RCC_APB2PeriphReset(uint8_t PerNum);
void RCC_GetAPB1Value(uint32_t *APB2Value);
void RCC_GetAPB2Value(uint32_t *APB2Value);
void RCC_GetSysClkType(uint8_t *SysClkType);
void RCC_GetSysClkValue(uint32_t *SysClkValue);


#endif /* STM32F407XX_RCC_DRIVER_H_ */
