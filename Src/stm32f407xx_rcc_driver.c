

/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: August 4,2024
 *      Author: tra vu
 */


#include "stm32f407xx_rcc_driver.h"

static void EnableHSE_crys(void);
static void EnableHSE_rc(void);
static void DisableHSE(void);
static void SelectHSE(void);
static void EnableHSI(void);
static void DisableHSI(void);
static void SelectHSI(void);

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = {2,4,8,16};



void RCC_SetAHBPrescaller(uint8_t AHBPrescaller)
{
	RCC->CFGR &= ~(0xF << 4);
	RCC->CFGR |= (AHBPrescaller << 4);         //AHB  pre-scaler
}

void RCC_SetAPB1Prescaller(uint8_t APB1Prescaller)
{
	RCC->CFGR &= ~(0x7 << 10);
	RCC->CFGR |= (APB1Prescaller << 10);      //APB1 pre-scaler
}

void RCC_SetAPB2Prescaller(uint8_t APB2Prescaller)
{
	RCC->CFGR &= ~(0x7 << 13);
	RCC->CFGR |= (APB2Prescaller << 13);      //APB2 pre-scaler
}



void RCC_SelectSystemClockSource(uint8_t SysClkSrc)
{
	if(SysClkSrc == RCC_SYSCLK_HSE_CRYSTAL)
	{
		EnableHSE_crys(); // enable the HSE
		SelectHSE();      // select HSE
		DisableHSI();     // disable HSI
	}
	else if(SysClkSrc == RCC_SYSCLK_HSE_RC)
	{
		EnableHSE_rc();    // enable the HSE
		SelectHSE();       // select HSE
		DisableHSI();      // disable HSI
	}
	else if(SysClkSrc == RCC_SYSCLK_HSI)
	{
		EnableHSI();      // enable the HSI
		SelectHSI();      // select HSI
		DisableHSE();     // disable HSE
	}
	else if(SysClkSrc == RCC_SYSCLK_PLL)
	{

	}
}


void RCC_AHB1PeriphClockEnable(uint8_t PerNum)
{
	RCC->AHB1ENR |= 1 << PerNum;
}


void RCC_AHB2PeriphClockEnable(uint8_t PerNum)
{
	RCC->AHB2ENR |= 1 << PerNum;
}


void RCC_APB1PeriphClockEnable(uint8_t PerNum)
{
	RCC->APB1ENR |= 1 << PerNum;

}


void RCC_APB2PeriphClockEnable(uint8_t PerNum)
{
	RCC->APB2ENR |= 1 << PerNum;
}


void RCC_AHB1PeriphClockDisable(uint8_t PerNum)
{
	RCC->AHB1ENR &= ~(1 << PerNum);
}


void RCC_AHB2PeriphClockDisable(uint8_t PerNum)
{
	RCC->AHB2ENR &= ~(1 << PerNum);
}


void RCC_APB1PeriphClockDisable(uint8_t PerNum)
{
	RCC->APB1ENR &= ~(1 << PerNum);
}

void RCC_AHB1PeriphReset(uint8_t PerNum)
{
	RCC->AHB1RSTR |= 1 << PerNum;
	RCC->AHB1RSTR &= ~(1 << PerNum);
}

void RCC_APB2PeriphClockDisable(uint8_t PerNum)
{
	RCC->APB2ENR &= ~(1 << PerNum);
}

void RCC_AHB2PeriphReset(uint8_t PerNum)
{
	RCC->AHB2RSTR |= 1 << PerNum;
	RCC->AHB2RSTR &= ~(1 << PerNum);
}


void RCC_APB1PeriphReset(uint8_t PerNum)
{
	RCC->APB1RSTR |= 1 << PerNum;
	RCC->APB1RSTR &= ~(1 << PerNum);
}


void RCC_APB2PeriphReset(uint8_t PerNum)
{
	RCC->APB2RSTR |= 1 << PerNum;
	RCC->APB2RSTR &= ~(1 << PerNum);
}


uint32_t RCC_GetAPB1Value(void)	//uint32_t *APB2Value
{
	uint32_t PCLK1,SystemClk;
	uint8_t ClockSource,Temp,APB1Prescaler;
	uint16_t AHBPrescaler;

	ClockSource = ((RCC->CFGR >> RCC_CFGR_SWS0) & 0x3);

	if(ClockSource == 0 )
	{
		SystemClk = 16000000; //HSI
	}
	else if(ClockSource == 1)
	{
		SystemClk = 8000000; //HSE
	}
	else if (ClockSource == 2)
	{
		// PLL
	}
	else if (ClockSource == 3) {
		//PLL_R used as the system clock
	}
	
	//for AHB pre-scaler
	Temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(Temp < 8)
	{
		AHBPrescaler = 1;
	}
	else
	{
		AHBPrescaler = AHB_PreScaler[Temp-8];
	}

	//apb1
	Temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(Temp < 4)
	{
		APB1Prescaler = 1;
	}
	else
	{
		APB1Prescaler = APB_PreScaler[Temp-4];
	}

	PCLK1 =  (SystemClk / AHBPrescaler) /APB1Prescaler;

//	*APB2Value = PCLK1;
	return PCLK1;
}

void RCC_GetAPB2Value(uint32_t *APB2Value)
{
	uint32_t PCLK1, SystemClk;

	uint8_t ClockSource, Temp, APB2Prescaler;
	uint16_t AHBPrescaler;

	ClockSource = ((RCC->CFGR >> RCC_CFGR_SWS0) & 0x3);

	if(ClockSource == 0 )
	{
		SystemClk = 16000000; //HSI
	}
	else if(ClockSource == 1)
	{
		SystemClk = 8000000; //HSE
	}
	else if (ClockSource == 2)
	{
		// PLL
	}

	//for AHB pre-scaler
	Temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(Temp < 8)
	{
		AHBPrescaler = 1;
	}
	else
	{
		AHBPrescaler = AHB_PreScaler[Temp-8];
	}

	//apb1
	Temp = ((RCC->CFGR >> 13 ) & 0x7);

	if(Temp < 4)
	{
		APB2Prescaler = 1;
	}
	else
	{
		APB2Prescaler = APB_PreScaler[Temp-4];
	}

	PCLK1 =  (SystemClk / APB2Prescaler) /AHBPrescaler;

	*APB2Value = PCLK1;
}

void RCC_GetSysClkValue(uint32_t *SysClkValue)
{
	uint8_t ClockSource = ((RCC->CFGR >> RCC_CFGR_SWS0) & 0x3);

	if(ClockSource == 0 )
	{
		*SysClkValue = 16000000; //HSI
	}
	else if(ClockSource == 1)
	{
		*SysClkValue = 8000000; //HSE
	}
	else if (ClockSource == 2)
	{
		// PLL
	}
}

void RCC_GetSysClkType(uint8_t *SysClkType)
{
	uint8_t ClockSource;

	ClockSource = ((RCC->CFGR >> RCC_CFGR_SWS0) & 0x3);

	if(ClockSource == 0 )
	{
		*SysClkType = RCC_SYSCLK_HSI; //HSI
	}
	else if(ClockSource == 1)
	{
		*SysClkType = RCC_SYSCLK_HSE_CRYSTAL; //HSE
	}
	else if (ClockSource == 2)
	{
		*SysClkType = RCC_SYSCLK_PLL; // PLL
	}
}


/****************************************** static functions ********************************************/

static void EnableHSE_crys(void)
{
	RCC->CR |= 1 << RCC_CR_HSEON;
	RCC->CR |= 1 << RCC_CR_HSYBYP;
}

static void EnableHSE_rc(void)
{
	RCC->CR |= 1 << RCC_CR_HSEON;
	RCC->CR |= 1 << RCC_CR_HSYBYP;
}

static void DisableHSE(void)
{
	RCC->CR &= ~(1 << RCC_CR_HSEON);
	RCC->CR &= ~(1 << RCC_CR_HSYBYP);
}

static void SelectHSE(void)
{
	RCC->CFGR |= 1 << RCC_CFGR_SW0;
	RCC->CFGR &= ~(1 << RCC_CFGR_SW1);
}


static void EnableHSI(void)
{
	RCC->CR |= 1 << RCC_CR_HSION;
	RCC->CR |= 1 << 7;		// HSITRIM[4:0]=0x8
}

static void DisableHSI(void)
{
	RCC->CR &= ~(1 << RCC_CR_HSION);
}

static void SelectHSI(void)
{
	RCC->CFGR &= ~(1 << RCC_CFGR_SW0);
	RCC->CFGR &= ~(1 << RCC_CFGR_SW1);
}

