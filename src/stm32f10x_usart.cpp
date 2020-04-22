/**
  ******************************************************************************
  * @file    stm32f10x_usart.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file provides all the USART firmware functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include <stdio.h>
#include <stdarg.h>
_ENDL endl;
uint16_t USART::ObjectNum[2] = {0,0};
/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup USART
  * @brief USART driver modules
  * @{
  */

/** @defgroup USART_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_Defines
  * @{
  */

#define CR1_UE_Set                ((uint16_t)0x2000)  /*!< USART Enable Mask */
#define CR1_UE_Reset              ((uint16_t)0xDFFF)  /*!< USART Disable Mask */

#define CR1_WAKE_Mask             ((uint16_t)0xF7FF)  /*!< USART WakeUp Method Mask */

#define CR1_RWU_Set               ((uint16_t)0x0002)  /*!< USART mute mode Enable Mask */
#define CR1_RWU_Reset             ((uint16_t)0xFFFD)  /*!< USART mute mode Enable Mask */
#define CR1_SBK_Set               ((uint16_t)0x0001)  /*!< USART Break Character send Mask */
#define CR1_CLEAR_Mask            ((uint16_t)0xE9F3)  /*!< USART CR1 Mask */
#define CR2_Address_Mask          ((uint16_t)0xFFF0)  /*!< USART address Mask */

#define CR2_LINEN_Set              ((uint16_t)0x4000)  /*!< USART LIN Enable Mask */
#define CR2_LINEN_Reset            ((uint16_t)0xBFFF)  /*!< USART LIN Disable Mask */

#define CR2_LBDL_Mask             ((uint16_t)0xFFDF)  /*!< USART LIN Break detection Mask */
#define CR2_STOP_CLEAR_Mask       ((uint16_t)0xCFFF)  /*!< USART CR2 STOP Bits Mask */
#define CR2_CLOCK_CLEAR_Mask      ((uint16_t)0xF0FF)  /*!< USART CR2 Clock Mask */

#define CR3_SCEN_Set              ((uint16_t)0x0020)  /*!< USART SC Enable Mask */
#define CR3_SCEN_Reset            ((uint16_t)0xFFDF)  /*!< USART SC Disable Mask */

#define CR3_NACK_Set              ((uint16_t)0x0010)  /*!< USART SC NACK Enable Mask */
#define CR3_NACK_Reset            ((uint16_t)0xFFEF)  /*!< USART SC NACK Disable Mask */

#define CR3_HDSEL_Set             ((uint16_t)0x0008)  /*!< USART Half-Duplex Enable Mask */
#define CR3_HDSEL_Reset           ((uint16_t)0xFFF7)  /*!< USART Half-Duplex Disable Mask */

#define CR3_IRLP_Mask             ((uint16_t)0xFFFB)  /*!< USART IrDA LowPower mode Mask */
#define CR3_CLEAR_Mask            ((uint16_t)0xFCFF)  /*!< USART CR3 Mask */

#define CR3_IREN_Set              ((uint16_t)0x0002)  /*!< USART IrDA Enable Mask */
#define CR3_IREN_Reset            ((uint16_t)0xFFFD)  /*!< USART IrDA Disable Mask */
#define GTPR_LSB_Mask             ((uint16_t)0x00FF)  /*!< Guard Time Register LSB Mask */
#define GTPR_MSB_Mask             ((uint16_t)0xFF00)  /*!< Guard Time Register MSB Mask */
#define IT_Mask                   ((uint16_t)0x001F)  /*!< USART Interrupt Mask */

/* USART OverSampling-8 Mask */
#define CR1_OVER8_Set             ((u16)0x8000)  /* USART OVER8 mode Enable Mask */
#define CR1_OVER8_Reset           ((u16)0x7FFF)  /* USART OVER8 mode Disable Mask */

/* USART One Bit Sampling Mask */
#define CR3_ONEBITE_Set           ((u16)0x0800)  /* USART ONEBITE mode Enable Mask */
#define CR3_ONEBITE_Reset         ((u16)0xF7FF)  /* USART ONEBITE mode Disable Mask */

/**
  * @}
  */

/** @defgroup USART_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */
USART::~USART()
{
	if(this->Decrease()==0)
	{
		this->Off();
		RCC::ClockCmd(*this,DISABLE);
	}
}
void USART::Increase()
{
	switch((uint32_t)USARTx)
	{
		case USART1_BASE:
			ObjectNum[0]++;
			break;
		case USART2_BASE:
			ObjectNum[1]++;
			break;
	}
}
uint16_t USART::Decrease(void)
{
	switch((uint32_t)USARTx)
	{
		case USART1_BASE:
			return --ObjectNum[0];
		case USART2_BASE:
			return --ObjectNum[1];
	}
	return 0xFFFF;
}
USART::USART(USART_TypeDef* _USARTx,uint32_t BaudRate,uint16_t WordLength,uint16_t StopBits,\
             uint16_t Parity,uint16_t Mode,uint16_t HardwareFlowControl):USARTx(_USARTx)
{
	USART::InitTypeDef aaa;
	this->Increase();
	aaa.BaudRate = BaudRate;
	aaa.WordLength = WordLength;
	aaa.StopBits = StopBits;
	aaa.Parity = Parity;
	aaa.Mode = Mode;
	aaa.HardwareFlowControl = HardwareFlowControl;
	this->Init(aaa);
	this->On();
}
USART::USART(const USART& Other)
{
	this->USARTx = Other.USARTx;
	this->Increase();
}
/** @defgroup USART_Private_Functions
  * @{
  */

/**
  * @brief  Deinitializes the USARTx peripheral registers to their default reset values.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *      USART1, USART2, USART3, UART4 or UART5.
  * @retval None
  */
void USART::DeInit(void)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));

	if(USARTx == USART1)
	{
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
	}
	else if(USARTx == USART2)
	{
		RCC::APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC::APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
	}
	else if(USARTx == USART3)
	{
		RCC::APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
		RCC::APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
	}
	else if(USARTx == UART4)
	{
		RCC::APB1PeriphResetCmd(RCC_APB1Periph_UART4, ENABLE);
		RCC::APB1PeriphResetCmd(RCC_APB1Periph_UART4, DISABLE);
	}
	else
	{
		if(USARTx == UART5)
		{
			RCC::APB1PeriphResetCmd(RCC_APB1Periph_UART5, ENABLE);
			RCC::APB1PeriphResetCmd(RCC_APB1Periph_UART5, DISABLE);
		}
	}
}

/**
  * @brief  Initializes the USARTx peripheral according to the specified
  *         parameters in the USART_InitStruct .
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *         that contains the configuration information for the specified USART
  *         peripheral.
  * @retval None
  */
void USART::Init(InitTypeDef &InitStruct)
{
	uint32_t tmpreg = 0x00, apbclock = 0x00;
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;
	uint32_t usartxbase = 0;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_BAUDRATE(InitStruct.BaudRate));
	assert_param(IS_USART_WORD_LENGTH(InitStruct.WordLength));
	assert_param(IS_USART_STOPBITS(InitStruct.StopBits));
	assert_param(IS_USART_PARITY(InitStruct.Parity));
	assert_param(IS_USART_MODE(InitStruct.Mode));
	assert_param(IS_USART_HARDWARE_FLOW_CONTROL(InitStruct.HardwareFlowControl));
	RCC::ClockCmd(*this,ENABLE);
	/* The hardware flow control is available only for USART1, USART2 and USART3 */
	if(InitStruct.HardwareFlowControl != USART_HardwareFlowControl_None)
	{
		assert_param(IS_USART_123_PERIPH(USARTx));
	}

	usartxbase = (uint32_t)USARTx;

	/*---------------------------- USART CR2 Configuration -----------------------*/
	tmpreg = USARTx->CR2;
	/* Clear STOP[13:12] bits */
	tmpreg &= CR2_STOP_CLEAR_Mask;
	/* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
	/* Set STOP[13:12] bits according to StopBits value */
	tmpreg |= (uint32_t)InitStruct.StopBits;

	/* Write to USART CR2 */
	USARTx->CR2 = (uint16_t)tmpreg;

	/*---------------------------- USART CR1 Configuration -----------------------*/
	tmpreg = USARTx->CR1;
	/* Clear M, PCE, PS, TE and RE bits */
	tmpreg &= CR1_CLEAR_Mask;
	/* Configure the USART Word Length, Parity and mode ----------------------- */
	/* Set the M bits according to WordLength value */
	/* Set PCE and PS bits according to Parity value */
	/* Set TE and RE bits according to Mode value */
	tmpreg |= (uint32_t)InitStruct.WordLength | InitStruct.Parity |
	          InitStruct.Mode;
	/* Write to USART CR1 */
	USARTx->CR1 = (uint16_t)tmpreg;

	/*---------------------------- USART CR3 Configuration -----------------------*/
	tmpreg = USARTx->CR3;
	/* Clear CTSE and RTSE bits */
	tmpreg &= CR3_CLEAR_Mask;
	/* Configure the USART HFC -------------------------------------------------*/
	/* Set CTSE and RTSE bits according to HardwareFlowControl value */
	tmpreg |= InitStruct.HardwareFlowControl;
	/* Write to USART CR3 */
	USARTx->CR3 = (uint16_t)tmpreg;

	/*---------------------------- USART BRR Configuration -----------------------*/
	/* Configure the USART Baud Rate -------------------------------------------*/
	RCC::GetClocksFreq(RCC_ClocksStatus);
	if(usartxbase == USART1_BASE)
	{
		apbclock = RCC_ClocksStatus.PCLK2_Frequency;
	}
	else
	{
		apbclock = RCC_ClocksStatus.PCLK1_Frequency;
	}

	/* Determine the integer part */
	if((USARTx->CR1 & CR1_OVER8_Set) != 0)
	{
		/* Integer part computing in case Oversampling mode is 8 Samples */
		integerdivider = ((25 * apbclock) / (2 * (InitStruct.BaudRate)));
	}
	else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
	{
		/* Integer part computing in case Oversampling mode is 16 Samples */
		integerdivider = ((25 * apbclock) / (4 * (InitStruct.BaudRate)));
	}
	tmpreg = (integerdivider / 100) << 4;

	/* Determine the fractional part */
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	/* Implement the fractional part in the register */
	if((USARTx->CR1 & CR1_OVER8_Set) != 0)
	{
		tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
	}
	else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
	{
		tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
	}

	/* Write to USART BRR */
	USARTx->BRR = (uint16_t)tmpreg;
}

/**
  * @brief  Fills each USART_InitStruct member with its default value.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
void USART::StructInit(InitTypeDef &InitStruct)
{
	/* USART_InitStruct members default value */
	InitStruct.BaudRate = 9600;
	InitStruct.WordLength = USART_WordLength_8b;
	InitStruct.StopBits = USART_StopBits_1;
	InitStruct.Parity = USART_Parity_No ;
	InitStruct.Mode = USART_Mode_Rx | USART_Mode_Tx;
	InitStruct.HardwareFlowControl = USART_HardwareFlowControl_None;
}

/**
  * @brief  Initializes the USARTx peripheral Clock according to the
  *          specified parameters in the USART_ClockInitStruct .
  * @param  USARTx: where x can be 1, 2, 3 to select the USART peripheral.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *         structure that contains the configuration information for the specified
  *         USART peripheral.
  * @note The Smart Card and Synchronous modes are not available for UART4 and UART5.
  * @retval None
  */
void USART::Init(ClockInitTypeDef &ClockInitStruct)
{
	uint32_t tmpreg = 0x00;
	/* Check the parameters */
	assert_param(IS_USART_123_PERIPH(USARTx));
	assert_param(IS_USART_CLOCK(ClockInitStruct.Clock));
	assert_param(IS_USART_CPOL(ClockInitStruct.CPOL));
	assert_param(IS_USART_CPHA(ClockInitStruct.CPHA));
	assert_param(IS_USART_LASTBIT(ClockInitStruct.LastBit));

	/*---------------------------- USART CR2 Configuration -----------------------*/
	tmpreg = USARTx->CR2;
	/* Clear CLKEN, CPOL, CPHA and LBCL bits */
	tmpreg &= CR2_CLOCK_CLEAR_Mask;
	/* Configure the USART Clock, CPOL, CPHA and LastBit ------------*/
	/* Set CLKEN bit according to Clock value */
	/* Set CPOL bit according to CPOL value */
	/* Set CPHA bit according to CPHA value */
	/* Set LBCL bit according to LastBit value */
	tmpreg |= (uint32_t)ClockInitStruct.Clock | ClockInitStruct.CPOL |
	          ClockInitStruct.CPHA | ClockInitStruct.LastBit;
	/* Write to USART CR2 */
	USARTx->CR2 = (uint16_t)tmpreg;
}

/**
  * @brief  Fills each USART_ClockInitStruct member with its default value.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
void USART::StructInit(ClockInitTypeDef &ClockInitStruct)
{
	/* USART_ClockInitStruct members default value */
	ClockInitStruct.Clock = USART_Clock_Disable;
	ClockInitStruct.CPOL = USART_CPOL_Low;
	ClockInitStruct.CPHA = USART_CPHA_1Edge;
	ClockInitStruct.LastBit = USART_LastBit_Disable;
}

/**
  * @brief  Enables or disables the specified USART peripheral.
  * @param  USARTx: Select the USART or the UART peripheral.
  *         This parameter can be one of the following values:
  *           USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USARTx peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART::Cmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		/* Enable the selected USART by setting the UE bit in the CR1 register */
		USARTx->CR1 |= CR1_UE_Set;
	}
	else
	{
		/* Disable the selected USART by clearing the UE bit in the CR1 register */
		USARTx->CR1 &= CR1_UE_Reset;
	}
}

/**
  * @brief  Enables or disables the specified USART interrupts.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TXE:  Transmit Data Register empty interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *     @arg USART_IT_IDLE: Idle line detection interrupt
  *     @arg USART_IT_PE:   Parity Error interrupt
  *     @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @param  NewState: new state of the specified USARTx interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART::ITConfig(uint16_t USART_IT, FunctionalState NewState)
{
	uint32_t usartreg = 0x00, itpos = 0x00, itmask = 0x00;
	uint32_t usartxbase = 0x00;
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_CONFIG_IT(USART_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	/* The CTS interrupt is not available for UART4 and UART5 */
	if(USART_IT == USART_IT_CTS)
	{
		assert_param(IS_USART_123_PERIPH(USARTx));
	}

	usartxbase = (uint32_t)USARTx;

	/* Get the USART register index */
	usartreg = (((uint8_t)USART_IT) >> 0x05);

	/* Get the interrupt position */
	itpos = USART_IT & IT_Mask;
	itmask = (((uint32_t)0x01) << itpos);

	if(usartreg == 0x01)  /* The IT is in CR1 register */
	{
		usartxbase += 0x0C;
	}
	else if(usartreg == 0x02)  /* The IT is in CR2 register */
	{
		usartxbase += 0x10;
	}
	else /* The IT is in CR3 register */
	{
		usartxbase += 0x14;
	}
	if(NewState != DISABLE)
	{
		*(__IO uint32_t*)usartxbase  |= itmask;
	}
	else
	{
		*(__IO uint32_t*)usartxbase &= ~itmask;
	}
}

/**
  * @brief  Enables or disables the USART’s DMA interface.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_DMAReq: specifies the DMA request.
  *   This parameter can be any combination of the following values:
  *     @arg USART_DMAReq_Tx: USART DMA transmit request
  *     @arg USART_DMAReq_Rx: USART DMA receive request
  * @param  NewState: new state of the DMA Request sources.
  *   This parameter can be: ENABLE or DISABLE.
  * @note The DMA mode is not available for UART5 except in the STM32
  *       High density value line devices(STM32F10X_HD_VL).
  * @retval None
  */
void USART::DMACmd(uint16_t USART_DMAReq, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_DMAREQ(USART_DMAReq));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	if(NewState != DISABLE)
	{
		/* Enable the DMA transfer for selected requests by setting the DMAT and/or
		   DMAR bits in the USART CR3 register */
		USARTx->CR3 |= USART_DMAReq;
	}
	else
	{
		/* Disable the DMA transfer for selected requests by clearing the DMAT and/or
		   DMAR bits in the USART CR3 register */
		USARTx->CR3 &= (uint16_t)~USART_DMAReq;
	}
}

/**
  * @brief  Sets the address of the USART node.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_Address: Indicates the address of the USART node.
  * @retval None
  */
void USART::SetAddress(uint8_t USART_Address)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_ADDRESS(USART_Address));

	/* Clear the USART address */
	USARTx->CR2 &= CR2_Address_Mask;
	/* Set the USART address node */
	USARTx->CR2 |= USART_Address;
}

/**
  * @brief  Selects the USART WakeUp method.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_WakeUp: specifies the USART wakeup method.
  *   This parameter can be one of the following values:
  *     @arg USART_WakeUp_IdleLine: WakeUp by an idle line detection
  *     @arg USART_WakeUp_AddressMark: WakeUp by an address mark
  * @retval None
  */
void USART::WakeUpConfig(uint16_t USART_WakeUp)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_WAKEUP(USART_WakeUp));

	USARTx->CR1 &= CR1_WAKE_Mask;
	USARTx->CR1 |= USART_WakeUp;
}

/**
  * @brief  Determines if the USART is in mute mode or not.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART mute mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART::ReceiverWakeUpCmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		/* Enable the USART mute mode  by setting the RWU bit in the CR1 register */
		USARTx->CR1 |= CR1_RWU_Set;
	}
	else
	{
		/* Disable the USART mute mode by clearing the RWU bit in the CR1 register */
		USARTx->CR1 &= CR1_RWU_Reset;
	}
}

/**
  * @brief  Sets the USART LIN Break detection length.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_LINBreakDetectLength: specifies the LIN break detection length.
  *   This parameter can be one of the following values:
  *     @arg USART_LINBreakDetectLength_10b: 10-bit break detection
  *     @arg USART_LINBreakDetectLength_11b: 11-bit break detection
  * @retval None
  */
void USART::LINBreakDetectLengthConfig(uint16_t USART_LINBreakDetectLength)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_LIN_BREAK_DETECT_LENGTH(USART_LINBreakDetectLength));

	USARTx->CR2 &= CR2_LBDL_Mask;
	USARTx->CR2 |= USART_LINBreakDetectLength;
}

/**
  * @brief  Enables or disables the USART’s LIN mode.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART LIN mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART::LINCmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		/* Enable the LIN mode by setting the LINEN bit in the CR2 register */
		USARTx->CR2 |= CR2_LINEN_Set;
	}
	else
	{
		/* Disable the LIN mode by clearing the LINEN bit in the CR2 register */
		USARTx->CR2 &= CR2_LINEN_Reset;
	}
}

/**
  * @brief  Transmits single data through the USARTx peripheral.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  Data: the data to transmit.
  * @retval None
  */
void USART::SendData(uint16_t Data)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_DATA(Data));

	/* Transmit Data */
	USARTx->DR = (Data & (uint16_t)0x01FF);
}

/**
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval The received data.
  */
uint16_t USART::ReceiveData(void)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));

	/* Receive Data */
	return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
}

/**
  * @brief  Transmits break characters.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval None
  */
void USART::SendBreak(void)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));

	/* Send break characters */
	USARTx->CR1 |= CR1_SBK_Set;
}

/**
  * @brief  Sets the specified USART guard time.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral.
  * @param  USART_GuardTime: specifies the guard time.
  * @note The guard time bits are not available for UART4 and UART5.
  * @retval None
  */
void USART::SetGuardTime(uint8_t USART_GuardTime)
{
	/* Check the parameters */
	assert_param(IS_USART_123_PERIPH(USARTx));

	/* Clear the USART Guard time */
	USARTx->GTPR &= GTPR_LSB_Mask;
	/* Set the USART guard time */
	USARTx->GTPR |= (uint16_t)((uint16_t)USART_GuardTime << 0x08);
}

/**
  * @brief  Sets the system clock prescaler.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_Prescaler: specifies the prescaler clock.
  * @note   The function is used for IrDA mode with UART4 and UART5.
  * @retval None
  */
void USART::SetPrescaler(uint8_t USART_Prescaler)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));

	/* Clear the USART prescaler */
	USARTx->GTPR &= GTPR_MSB_Mask;
	/* Set the USART prescaler */
	USARTx->GTPR |= USART_Prescaler;
}

/**
  * @brief  Enables or disables the USART’s Smart Card mode.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral.
  * @param  NewState: new state of the Smart Card mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note The Smart Card mode is not available for UART4 and UART5.
  * @retval None
  */
void USART::SmartCardCmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_123_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	if(NewState != DISABLE)
	{
		/* Enable the SC mode by setting the SCEN bit in the CR3 register */
		USARTx->CR3 |= CR3_SCEN_Set;
	}
	else
	{
		/* Disable the SC mode by clearing the SCEN bit in the CR3 register */
		USARTx->CR3 &= CR3_SCEN_Reset;
	}
}

/**
  * @brief  Enables or disables NACK transmission.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral.
  * @param  NewState: new state of the NACK transmission.
  *   This parameter can be: ENABLE or DISABLE.
  * @note The Smart Card mode is not available for UART4 and UART5.
  * @retval None
  */
void USART::SmartCardNACKCmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_123_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	if(NewState != DISABLE)
	{
		/* Enable the NACK transmission by setting the NACK bit in the CR3 register */
		USARTx->CR3 |= CR3_NACK_Set;
	}
	else
	{
		/* Disable the NACK transmission by clearing the NACK bit in the CR3 register */
		USARTx->CR3 &= CR3_NACK_Reset;
	}
}

/**
  * @brief  Enables or disables the USART’s Half Duplex communication.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART Communication.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART::HalfDuplexCmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		/* Enable the Half-Duplex mode by setting the HDSEL bit in the CR3 register */
		USARTx->CR3 |= CR3_HDSEL_Set;
	}
	else
	{
		/* Disable the Half-Duplex mode by clearing the HDSEL bit in the CR3 register */
		USARTx->CR3 &= CR3_HDSEL_Reset;
	}
}


/**
  * @brief  Enables or disables the USART's 8x oversampling mode.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART one bit sampling method.
  *   This parameter can be: ENABLE or DISABLE.
  * @note
  *     This function has to be called before calling USART_Init()
  *     function in order to have correct baudrate Divider value.
  * @retval None
  */
void USART::OverSampling8Cmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		/* Enable the 8x Oversampling mode by setting the OVER8 bit in the CR1 register */
		USARTx->CR1 |= CR1_OVER8_Set;
	}
	else
	{
		/* Disable the 8x Oversampling mode by clearing the OVER8 bit in the CR1 register */
		USARTx->CR1 &= CR1_OVER8_Reset;
	}
}

/**
  * @brief  Enables or disables the USART's one bit sampling method.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART one bit sampling method.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART::OneBitMethodCmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		/* Enable the one bit method by setting the ONEBITE bit in the CR3 register */
		USARTx->CR3 |= CR3_ONEBITE_Set;
	}
	else
	{
		/* Disable tthe one bit method by clearing the ONEBITE bit in the CR3 register */
		USARTx->CR3 &= CR3_ONEBITE_Reset;
	}
}

/**
  * @brief  Configures the USART's IrDA interface.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IrDAMode: specifies the IrDA mode.
  *   This parameter can be one of the following values:
  *     @arg USART_IrDAMode_LowPower
  *     @arg USART_IrDAMode_Normal
  * @retval None
  */
void USART::IrDAConfig(uint16_t USART_IrDAMode)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_IRDA_MODE(USART_IrDAMode));

	USARTx->CR3 &= CR3_IRLP_Mask;
	USARTx->CR3 |= USART_IrDAMode;
}

/**
  * @brief  Enables or disables the USART's IrDA interface.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the IrDA mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART::IrDACmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		/* Enable the IrDA mode by setting the IREN bit in the CR3 register */
		USARTx->CR3 |= CR3_IREN_Set;
	}
	else
	{
		/* Disable the IrDA mode by clearing the IREN bit in the CR3 register */
		USARTx->CR3 &= CR3_IREN_Reset;
	}
}

/**
  * @brief  Checks whether the specified USART flag is set or not.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg USART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5)
  *     @arg USART_FLAG_LBD:  LIN Break detection flag
  *     @arg USART_FLAG_TXE:  Transmit data register empty flag
  *     @arg USART_FLAG_TC:   Transmission Complete flag
  *     @arg USART_FLAG_RXNE: Receive data register not empty flag
  *     @arg USART_FLAG_IDLE: Idle Line detection flag
  *     @arg USART_FLAG_ORE:  OverRun Error flag
  *     @arg USART_FLAG_NE:   Noise Error flag
  *     @arg USART_FLAG_FE:   Framing Error flag
  *     @arg USART_FLAG_PE:   Parity Error flag
  * @retval The new state of USART_FLAG (SET or RESET).
  */
FlagStatus USART::GetFlagStatus(uint16_t USART_FLAG)
{
	FlagStatus bitstatus = RESET;
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_FLAG(USART_FLAG));
	/* The CTS flag is not available for UART4 and UART5 */
	if(USART_FLAG == USART_FLAG_CTS)
	{
		assert_param(IS_USART_123_PERIPH(USARTx));
	}

	if((USARTx->SR & USART_FLAG) != (uint16_t)RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}

/**
  * @brief  Clears the USARTx's pending flags.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg USART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5).
  *     @arg USART_FLAG_LBD:  LIN Break detection flag.
  *     @arg USART_FLAG_TC:   Transmission Complete flag.
  *     @arg USART_FLAG_RXNE: Receive data register not empty flag.
  *
  * @note
  *   - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun
  *     error) and IDLE (Idle line detected) flags are cleared by software
  *     sequence: a read operation to USART_SR register (USART_GetFlagStatus())
  *     followed by a read operation to USART_DR register (USART_ReceiveData()).
  *   - RXNE flag can be also cleared by a read to the USART_DR register
  *     (USART_ReceiveData()).
  *   - TC flag can be also cleared by software sequence: a read operation to
  *     USART_SR register (USART_GetFlagStatus()) followed by a write operation
  *     to USART_DR register (USART_SendData()).
  *   - TXE flag is cleared only by a write to the USART_DR register
  *     (USART_SendData()).
  * @retval None
  */
void USART::ClearFlag(uint16_t USART_FLAG)
{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_CLEAR_FLAG(USART_FLAG));
	/* The CTS flag is not available for UART4 and UART5 */
	if((USART_FLAG & USART_FLAG_CTS) == USART_FLAG_CTS)
	{
		assert_param(IS_USART_123_PERIPH(USARTx));
	}

	USARTx->SR = (uint16_t)~USART_FLAG;
}

/**
  * @brief  Checks whether the specified USART interrupt has occurred or not.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TXE:  Tansmit Data Register empty interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *     @arg USART_IT_IDLE: Idle line detection interrupt
  *     @arg USART_IT_ORE:  OverRun Error interrupt
  *     @arg USART_IT_NE:   Noise Error interrupt
  *     @arg USART_IT_FE:   Framing Error interrupt
  *     @arg USART_IT_PE:   Parity Error interrupt
  * @retval The new state of USART_IT (SET or RESET).
  */
ITStatus USART::GetITStatus(uint16_t USART_IT)
{
	uint32_t bitpos = 0x00, itmask = 0x00, usartreg = 0x00;
	ITStatus bitstatus = RESET;
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_GET_IT(USART_IT));
	/* The CTS interrupt is not available for UART4 and UART5 */
	if(USART_IT == USART_IT_CTS)
	{
		assert_param(IS_USART_123_PERIPH(USARTx));
	}

	/* Get the USART register index */
	usartreg = (((uint8_t)USART_IT) >> 0x05);
	/* Get the interrupt position */
	itmask = USART_IT & IT_Mask;
	itmask = (uint32_t)0x01 << itmask;

	if(usartreg == 0x01)  /* The IT  is in CR1 register */
	{
		itmask &= USARTx->CR1;
	}
	else if(usartreg == 0x02)  /* The IT  is in CR2 register */
	{
		itmask &= USARTx->CR2;
	}
	else /* The IT  is in CR3 register */
	{
		itmask &= USARTx->CR3;
	}

	bitpos = USART_IT >> 0x08;
	bitpos = (uint32_t)0x01 << bitpos;
	bitpos &= USARTx->SR;
	if((itmask != (uint16_t)RESET)&&(bitpos != (uint16_t)RESET))
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}

	return bitstatus;
}

/**
  * @brief  Clears the USARTx's interrupt pending bits.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt.
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt.
  *
  * @note
  *   - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun
  *     error) and IDLE (Idle line detected) pending bits are cleared by
  *     software sequence: a read operation to USART_SR register
  *     (USART_GetITStatus()) followed by a read operation to USART_DR register
  *     (USART_ReceiveData()).
  *   - RXNE pending bit can be also cleared by a read to the USART_DR register
  *     (USART_ReceiveData()).
  *   - TC pending bit can be also cleared by software sequence: a read
  *     operation to USART_SR register (USART_GetITStatus()) followed by a write
  *     operation to USART_DR register (USART_SendData()).
  *   - TXE pending bit is cleared only by a write to the USART_DR register
  *     (USART_SendData()).
  * @retval None
  */
void USART::ClearITPendingBit(uint16_t USART_IT)
{
	uint16_t bitpos = 0x00, itmask = 0x00;
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_CLEAR_IT(USART_IT));
	/* The CTS interrupt is not available for UART4 and UART5 */
	if(USART_IT == USART_IT_CTS)
	{
		assert_param(IS_USART_123_PERIPH(USARTx));
	}

	bitpos = USART_IT >> 0x08;
	itmask = ((uint16_t)0x01 << (uint16_t)bitpos);
	USARTx->SR = (uint16_t)~itmask;
}
/**
  * @}
  */

/**
  * @}
  */

USART::USART(USART_TypeDef* _USARTx, InitTypeDef& InitStruct) : USARTx(_USARTx)
{
	this->Increase();
	this->Init(InitStruct);
	this->On();
}

/**
  * @}
  */
void USART::Send(unsigned char *Data,uint32_t Length)
{
	while(Length--)
	{
		while((USARTx -> SR & 0X40) == 0);
		USARTx->DR = (u8) *Data++;
	}
}
void USART::SendString(const char *Str)
{
	while(*Str)
	{
		while((USARTx -> SR & 0X40) == 0);
		USARTx->DR = (u8) *Str++;
	}
}
USART& USART::operator<<(bool b)
{
	return *this << (b ? "true" : "false");
}
USART& USART::operator << (char Data)
{
	while((USARTx -> SR & 0X40) == 0);
	USARTx->DR = (u8) Data;
	return *this;
}
USART& USART::operator << (unsigned char Data)
{
	char buf[10];
	snprintf(buf,sizeof(buf),"%#X",Data);
	SendString(buf);
	return *this;
}

USART& USART::operator << (short Data)
{
	char buf[20];
	snprintf(buf,sizeof(buf),"%d",Data);
	SendString(buf);
	return *this;
}
USART& USART::operator << (unsigned short Data)
{
	char buf[20];
	snprintf(buf,sizeof(buf),"%u",Data);
	SendString(buf);
	return *this;
}

USART& USART::operator << (int Data)
{
	char buf[20];
	snprintf(buf,sizeof(buf),"%d",Data);
	SendString(buf);
	return *this;
}
USART& USART::operator << (unsigned int Data)
{
	char buf[20];
	snprintf(buf,sizeof(buf),"%u",Data);
	SendString(buf);
	return *this;
}

USART& USART::operator << (long Data)
{
	char buf[20];
	snprintf(buf,sizeof(buf),"%ld",Data);
	SendString(buf);
	return *this;
}
USART& USART::operator << (unsigned long Data)
{
	char buf[20];
	snprintf(buf,sizeof(buf),"%lu",Data);
	SendString(buf);
	return *this;
}

USART& USART::operator << (float Data)
{
	char buf[20];
	snprintf(buf,sizeof(buf),"%f",Data);
	SendString(buf);
	return *this;
}
USART& USART::operator << (double Data)
{
	char buf[20];
	snprintf(buf,sizeof(buf),"%lf",Data);
	SendString(buf);
	return *this;
}
void USART::On()
{
	this->Cmd(ENABLE);
}
void USART::Off()
{
	this->Cmd(DISABLE);
}
uint32_t USART::Printf(const char *Format,...)
{
	va_list ap;
	uint32_t length;
	va_start(ap,Format);
	char buf[256];
	length = vsnprintf(buf,sizeof(buf),Format,ap);
	SendString(buf);
	va_end(ap);
	return length;
}
USART_TypeDef* USART::GetPeriphPointer(void)
{
	return this->USARTx;
}
USART& USART::operator << (void* p)
{
	char buf[11];
	snprintf(buf,sizeof(buf),"%#X",(uint32_t)p);
	SendString(buf);
	return *this;
}
USART& USART::operator<<(const char* Str)
{
	SendString(Str);
	return *this;
}
USART& USART::operator<<(_ENDL& a)
{
	*this << "\r\n";
	return *this;
}
USART& USART::operator>>(uint16_t& Data)
{
	Data = (uint16_t)this->USARTx->DR;
	return *this;
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
