


#include "stm32f407xx_spi_driver.h"
#include "stdio.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

//**************SOME HELPER FUNCTION IMPLEMENTATIONS START**************//

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	//Check the DFF bit in CR1
   if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
	   //16 bits DFF and load data into the DR
	   pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
	   pSPIHandle->TxLen--;
	   pSPIHandle->TxLen--;
	   (uint16_t*)pSPIHandle->pTxBuffer++;
   }
   else {
	   //8 bits DFF and load data into the DR
	   pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
	   pSPIHandle->TxLen--;
	   pSPIHandle->pTxBuffer++;
   }

   if(!pSPIHandle->TxLen) {
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
	   //16 bits DFF and load data into the buffer
	   *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
	   pSPIHandle->RxLen--;
	   pSPIHandle->RxLen--;
	   pSPIHandle->pRxBuffer++;
	   pSPIHandle->pRxBuffer++;
	}
	else {
	   //8 bits DFF and load data into the buffer
	   *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
	   pSPIHandle->RxLen--;;
	   pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen) {
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE){
		if (pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

 void SPI_Init(SPI_Handle_t *pSPIHandle) {
	 uint32_t temp = 0;
	 SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	 //configure the device mode
	 temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	 //configure the bus
	 if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		 //BIDIMODE mode should be clear
		 temp &= ~(1 << SPI_CR1_BIDIMODE);
	 }
	 else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		 //BIDIMODE mode should be set
		 temp |= (1 << SPI_CR1_BIDIMODE);
	 }
	 else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		 //BIDIMODE mode should be clear
		 temp &= ~(1 << SPI_CR1_BIDIMODE);
		 //RXONLY mode should be set
		 temp |= (1 << SPI_CR1_RXONLY);
	 }
	 //configure the SPI serial clock speed
	 temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	 //configure the DFF mode
	 temp |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	 //configure the CPOL mode
	 temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	 //configure the CPHA mode
	 temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	 //configure the SSM mode
	 temp |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	 pSPIHandle->pSPIx->CR1 = temp;
 }

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
}

void SPI_TransmitData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length) {
	while (Length > 0) {
		//Wait until the flag set
	   while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

	   //Check the DFF bit in CR1
	   if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
		   //16 bits DFF and load data into the DR
		   pSPIx->DR = *((uint16_t*)pTxBuffer);
		   Length--;
		   Length--;
		   (uint16_t*)pTxBuffer++;
	   }
	   else {
		   //8 bits DFF and load data into the DR
		   pSPIx->DR = *pTxBuffer;
		   Length--;
		   pTxBuffer++;
	   }
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length) {
	while (Length > 0) {
		//Wait until the flag set
	   while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

	   //Check the DFF bit in CR1
	   if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
		   //16 bits DFF and load data into the buffer
		   *((uint16_t*)pRxBuffer) = pSPIx->DR;
		   Length--;
		   Length--;
		   ((uint16_t*)pRxBuffer++);
	   }
	   else {
		   //8 bits DFF and load data into the buffer
		   *pRxBuffer = pSPIx->DR;
		   Length--;
		   pRxBuffer++;
	   }
	}
}

uint8_t SPI_TransmitDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length) {
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_RX) {
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer 	= pTxBuffer;
		pSPIHandle->TxLen		= Length;
		//2.  Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState		= SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Lenghth) {
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer	= pRxBuffer;
		pSPIHandle->RxLen 		= Lenghth;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState		= SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

	}
	return state;
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	 if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64);
		}
	 } else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= ( 1 << IRQNumber % 64);
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	 uint8_t iprx, iprxSection, shiftAmount;
	 iprx = IRQNumber / 4;
	 iprxSection = IRQNumber % 4;
	 shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	 *(NVIC_PR_BASE_ADDR + iprx ) = (IRQPriority << shiftAmount);
}

void SPI_IRQHandling(SPI_Handle_t *pHandle) {
	uint8_t temp1 , temp2;

	//First lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//Handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//Check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//Handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// Check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//Handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else {
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv) {

	//This is a weak implementation . the user application may override this function.
}



