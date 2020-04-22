/**
  ******************************************************************************
  * @file    stm32f10x_gpio.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file provides all the GPIO firmware functions.
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
#include "stm32f10x_gpio.h"
uint16_t GPIO::ObjectNum[7] = {0,0,0,0,0,0,0};
/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup GPIO
  * @brief GPIO driver modules
  * @{
  */

/** @defgroup GPIO_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup GPIO_Private_Defines
  * @{
  */

/* ------------ RCC registers bit address in the alias region ----------------*/
#define AFIO_OFFSET                 (AFIO_BASE - PERIPH_BASE)

/* --- EVENTCR Register -----*/

/* Alias word address of EVOE bit */
#define EVCR_OFFSET                 (AFIO_OFFSET + 0x00)
#define EVOE_BitNumber              ((uint8_t)0x07)
#define EVCR_EVOE_BB                (PERIPH_BB_BASE + (EVCR_OFFSET * 32) + (EVOE_BitNumber * 4))


/* ---  MAPR Register ---*/
/* Alias word address of MII_RMII_SEL bit */
#define MAPR_OFFSET                 (AFIO_OFFSET + 0x04)
#define MII_RMII_SEL_BitNumber      ((u8)0x17)
#define MAPR_MII_RMII_SEL_BB        (PERIPH_BB_BASE + (MAPR_OFFSET * 32) + (MII_RMII_SEL_BitNumber * 4))


#define EVCR_PORTPINCONFIG_MASK     ((uint16_t)0xFF80)
#define LSB_MASK                    ((uint16_t)0xFFFF)
#define DBGAFR_POSITION_MASK        ((uint32_t)0x000F0000)
#define DBGAFR_SWJCFG_MASK          ((uint32_t)0xF0FFFFFF)
#define DBGAFR_LOCATION_MASK        ((uint32_t)0x00200000)
#define DBGAFR_NUMBITS_MASK         ((uint32_t)0x00100000)
/**
  * @}
  */

/** @defgroup GPIO_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup GPIO_Private_Variables
  * @{
  */

/**
  * @}
  */

void GPIO::Increase(void)
{
	switch((uint32_t)GPIOx)
	{
		case GPIOA_BASE:
			ObjectNum[0]++;
			break;
		case GPIOB_BASE:
			ObjectNum[1]++;
			break;
		case GPIOC_BASE:
			ObjectNum[2]++;
			break;
		case GPIOD_BASE:
			ObjectNum[3]++;
			break;
		case GPIOE_BASE:
			ObjectNum[4]++;
			break;
		case GPIOF_BASE:
			ObjectNum[5]++;
			break;
		case GPIOG_BASE:
			ObjectNum[6]++;
			break;
	}
}

uint16_t GPIO::Decrease(void)
{
	switch((uint32_t)GPIOx)
	{
		case GPIOA_BASE:
			return --ObjectNum[0];
		case GPIOB_BASE:
			return --ObjectNum[1];
		case GPIOC_BASE:
			return --ObjectNum[2];
		case GPIOD_BASE:
			return --ObjectNum[3];
		case GPIOE_BASE:
			return --ObjectNum[4];
		case GPIOF_BASE:
			return --ObjectNum[5];
		case GPIOG_BASE:
			return --ObjectNum[6];
	}
	return 0xFFFF;
}
uint8_t GPIO::GetSource(void)
{
	switch((uint32_t)this->GPIOx)
	{
		case GPIOA_BASE:
			return GPIO_PortSourceGPIOA;
		case GPIOB_BASE:
			return GPIO_PortSourceGPIOB;
		case GPIOC_BASE:
			return GPIO_PortSourceGPIOC;
		case GPIOD_BASE:
			return GPIO_PortSourceGPIOD;
		case GPIOE_BASE:
			return GPIO_PortSourceGPIOE;
		case GPIOF_BASE:
			return GPIO_PortSourceGPIOF;
		case GPIOG_BASE:
			return GPIO_PortSourceGPIOG;
		default:
			return 255;

	}
}
/** @defgroup GPIO_Private_FunctionPrototypes
  * @{
  */
GPIO::~GPIO()
{
	if(this->Decrease()==0)
		RCC::ClockCmd(*this,DISABLE);
}
/**
  * @}
  */
GPIO::GPIO(GPIO_TypeDef* _GPIOx,InitTypeDef &InitStruct):GPIOx(_GPIOx)
{
	this->Increase();
	this->Init(InitStruct);
}
GPIO::GPIO(const GPIO& Other)
{
	this->GPIOx = Other.GPIOx;
	this->Increase();
}
void GPIO::operator<<=(uint16_t Data)
{
	(this->GPIOx->ODR) <<= Data;
}
void GPIO::operator>>=(uint16_t Data)
{
	(this->GPIOx->IDR) >>= Data;
}
GPIO& GPIO::operator=(uint16_t Data)
{
	(this->GPIOx->ODR) = Data;
	return *this;
}
GPIO::operator uint16_t()
{
	return this->GPIOx->IDR;
}
bool GPIO::operator==(uint16_t Data)
{
	return (bool)(Data == (this->GPIOx->IDR));
}
uint16_t GPIO::operator*(void)
{
	return this->GPIOx->ODR;
}
uint16_t GPIO::operator~(void)
{
	return ~(uint16_t)*this;
}
GPIO_Bit GPIO::operator[](uint8_t Pin)
{
	return GPIO_Bit(this->GPIOx, Pin);
}
GPIO_TypeDef* GPIO::GetPeriphPointer(void)
{
	return this->GPIOx;
}
/** @defgroup GPIO_Private_Functions
  * @{
  */
/**
  * @brief  Deinitializes the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval None
  */
void GPIO::DeInit(void)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

	if(GPIOx == GPIOA)
	{
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, DISABLE);
	}
	else if(GPIOx == GPIOB)
	{
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, ENABLE);
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, DISABLE);
	}
	else if(GPIOx == GPIOC)
	{
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, ENABLE);
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, DISABLE);
	}
	else if(GPIOx == GPIOD)
	{
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, ENABLE);
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, DISABLE);
	}
	else if(GPIOx == GPIOE)
	{
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOE, ENABLE);
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOE, DISABLE);
	}
	else if(GPIOx == GPIOF)
	{
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOF, ENABLE);
		RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOF, DISABLE);
	}
	else
	{
		if(GPIOx == GPIOG)
		{
			RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOG, ENABLE);
			RCC::APB2PeriphResetCmd(RCC_APB2Periph_GPIOG, DISABLE);
		}
	}
}

/**
  * @brief  Deinitializes the Alternate Functions (remap, event control
  *   and EXTI configuration) registers to their default reset values.
  * @param  None
  * @retval None
  */
void GPIO::AFIODeInit(void)
{
	RCC::APB2PeriphResetCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC::APB2PeriphResetCmd(RCC_APB2Periph_AFIO, DISABLE);
}

/**
  * @brief  Initializes the GPIOx peripheral according to the specified
  *         parameters in the GPIO_InitStruct.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that
  *         contains the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void GPIO::Init(InitTypeDef &InitStruct)
{
	uint32_t currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
	uint32_t tmpreg = 0x00, pinmask = 0x00;
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_MODE(InitStruct.Mode));
	assert_param(IS_GPIO_PIN(InitStruct.Pin));
	RCC::ClockCmd(*this,ENABLE);
	/*---------------------------- GPIO Mode Configuration -----------------------*/
	currentmode = ((uint32_t)InitStruct.Mode) & ((uint32_t)0x0F);
	if((((uint32_t)InitStruct.Mode) & ((uint32_t)0x10)) != 0x00)
	{
		/* Check the parameters */
		assert_param(IS_GPIO_SPEED(InitStruct.Speed));
		/* Output mode */
		currentmode |= (uint32_t)InitStruct.Speed;
	}
	/*---------------------------- GPIO CRL Configuration ------------------------*/
	/* Configure the eight low port pins */
	if(((uint32_t)InitStruct.Pin & ((uint32_t)0x00FF)) != 0x00)
	{
		tmpreg = GPIOx->CRL;
		for(pinpos = 0x00; pinpos < 0x08; pinpos++)
		{
			pos = ((uint32_t)0x01) << pinpos;
			/* Get the port pins position */
			currentpin = (InitStruct.Pin) & pos;
			if(currentpin == pos)
			{
				pos = pinpos << 2;
				/* Clear the corresponding low control register bits */
				pinmask = ((uint32_t)0x0F) << pos;
				tmpreg &= ~pinmask;
				/* Write the mode configuration in the corresponding bits */
				tmpreg |= (currentmode << pos);
				/* Reset the corresponding ODR bit */
				if(InitStruct.Mode == Mode_IPD)
				{
					GPIOx->BRR = (((uint32_t)0x01) << pinpos);
				}
				else
				{
					/* Set the corresponding ODR bit */
					if(InitStruct.Mode == Mode_IPU)
					{
						GPIOx->BSRR = (((uint32_t)0x01) << pinpos);
					}
				}
			}
		}
		GPIOx->CRL = tmpreg;
	}
	/*---------------------------- GPIO CRH Configuration ------------------------*/
	/* Configure the eight high port pins */
	if(InitStruct.Pin > 0x00FF)
	{
		tmpreg = GPIOx->CRH;
		for(pinpos = 0x00; pinpos < 0x08; pinpos++)
		{
			pos = (((uint32_t)0x01) << (pinpos + 0x08));
			/* Get the port pins position */
			currentpin = ((InitStruct.Pin) & pos);
			if(currentpin == pos)
			{
				pos = pinpos << 2;
				/* Clear the corresponding high control register bits */
				pinmask = ((uint32_t)0x0F) << pos;
				tmpreg &= ~pinmask;
				/* Write the mode configuration in the corresponding bits */
				tmpreg |= (currentmode << pos);
				/* Reset the corresponding ODR bit */
				if(InitStruct.Mode == Mode_IPD)
				{
					GPIOx->BRR = (((uint32_t)0x01) << (pinpos + 0x08));
				}
				/* Set the corresponding ODR bit */
				if(InitStruct.Mode == Mode_IPU)
				{
					GPIOx->BSRR = (((uint32_t)0x01) << (pinpos + 0x08));
				}
			}
		}
		GPIOx->CRH = tmpreg;
	}
}

/**
  * @brief  Fills each GPIO_InitStruct member with its default value.
  * @param  GPIO_InitStruct : pointer to a GPIO_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void GPIO::StructInit(InitTypeDef &InitStruct)
{
	/* Reset GPIO init structure parameters values */
	InitStruct.Pin  = Pin_All;
	InitStruct.Speed = Speed_2MHz;
	InitStruct.Mode = Mode_IN_FLOATING;
}

/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  Pin:  specifies the port bit to read.
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The input port pin value.
  */
bool GPIO::ReadInputDataBit(uint16_t Pin)
{

	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GET_GPIO_PIN(Pin));

	if((GPIOx->IDR & Pin) != (uint32_t)Bit_RESET)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
  * @brief  Reads the specified GPIO input data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval GPIO input data port value.
  */
uint16_t GPIO::ReadInputData(void)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

	return ((uint16_t)GPIOx->IDR);
}

/**
  * @brief  Reads the specified output data port bit.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  Pin:  specifies the port bit to read.
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The output port pin value.
  */
bool GPIO::ReadOutputDataBit(uint16_t Pin)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GET_GPIO_PIN(Pin));

	if((GPIOx->ODR & Pin) != (uint32_t)Bit_RESET)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
  * @brief  Reads the specified GPIO output data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval GPIO output data port value.
  */
uint16_t GPIO::ReadOutputData(void)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

	return ((uint16_t)GPIOx->ODR);
}

/**
  * @brief  Sets the selected data port bits.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  Pin: specifies the port bits to be written.
  *   This parameter can be any combination of x where x can be (0..15).
  * @retval None
  */
void GPIO::SetBits(uint16_t Pin)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(Pin));

	GPIOx->BSRR = (1<<Pin);
}

/**
  * @brief  Clears the selected data port bits.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  Pin: specifies the port bits to be written.
  *   This parameter can be any combination of x where x can be (0..15).
  * @retval None
  */
void GPIO::ResetBits(uint16_t Pin)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(Pin));

	GPIOx->BRR = (1<<Pin);
}

/**
  * @brief  Sets or clears the selected data port bit.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  Pin: specifies the port bit to be written.
  *   This parameter can be one of GPIO_Pin_x where x can be (0..15).
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be one of the BitAction enum values:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
void GPIO::WriteBit(uint16_t Pin, bool BitVal)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GET_GPIO_PIN(Pin));
	assert_param(IS_GPIO_BIT_ACTION(BitVal));

	if(BitVal != true)
	{
		GPIOx->BSRR = (1<<Pin);
	}
	else
	{
		GPIOx->BRR = (1<<Pin);
	}
}

/**
  * @brief  Writes data to the specified GPIO data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  PortVal: specifies the value to be written to the port output data register.
  * @retval None
  */
void GPIO::Write(uint16_t PortVal)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

	GPIOx->ODR = PortVal;
}

/**
  * @brief  Locks GPIO Pins configuration registers.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  Pin: specifies the port bit to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
  * @retval None
  */
void GPIO::PinLockConfig(uint16_t Pin)
{
	uint32_t tmp = 0x00010000;

	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(Pin));

	tmp |= Pin;
	/* Set LCKK bit */
	GPIOx->LCKR = tmp;
	/* Reset LCKK bit */
	GPIOx->LCKR =  Pin;
	/* Set LCKK bit */
	GPIOx->LCKR = tmp;
	/* Read LCKK bit*/
	tmp = GPIOx->LCKR;
	/* Read LCKK bit*/
	tmp = GPIOx->LCKR;
}

/**
  * @brief  Selects the GPIO pin used as Event output.
  * @param  GPIO_PortSource: selects the GPIO port to be used as source
  *   for Event output.
  *   This parameter can be GPIO_PortSourceGPIOx where x can be (A..E).
  * @param  GPIO_PinSource: specifies the pin for the Event output.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @retval None
  */
void GPIO::EventOutputConfig(uint8_t GPIO_PinSource)
{
	uint32_t tmpreg = 0x00;
	uint8_t GPIO_PortSource = this->GetSource();
	/* Check the parameters */
	assert_param(IS_GPIO_EVENTOUT_PORT_SOURCE(GPIO_PortSource));
	assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));

	tmpreg = AFIO->EVCR;
	/* Clear the PORT[6:4] and PIN[3:0] bits */
	tmpreg &= EVCR_PORTPINCONFIG_MASK;
	tmpreg |= (uint32_t)GPIO_PortSource << 0x04;
	tmpreg |= GPIO_PinSource;
	AFIO->EVCR = tmpreg;
}

/**
  * @brief  Enables or disables the Event Output.
  * @param  NewState: new state of the Event output.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void GPIO::EventOutputCmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	*(__IO uint32_t *) EVCR_EVOE_BB = (uint32_t)NewState;
}

/**
  * @brief  Changes the mapping of the specified pin.
  * @param  GPIO_Remap: selects the pin to remap.
  *   This parameter can be one of the following values:
  *     @arg GPIO_Remap_SPI1             : SPI1 Alternate Function mapping
  *     @arg GPIO_Remap_I2C1             : I2C1 Alternate Function mapping
  *     @arg GPIO_Remap_USART1           : USART1 Alternate Function mapping
  *     @arg GPIO_Remap_USART2           : USART2 Alternate Function mapping
  *     @arg GPIO_PartialRemap_USART3    : USART3 Partial Alternate Function mapping
  *     @arg GPIO_FullRemap_USART3       : USART3 Full Alternate Function mapping
  *     @arg GPIO_PartialRemap_TIM1      : TIM1 Partial Alternate Function mapping
  *     @arg GPIO_FullRemap_TIM1         : TIM1 Full Alternate Function mapping
  *     @arg GPIO_PartialRemap1_TIM2     : TIM2 Partial1 Alternate Function mapping
  *     @arg GPIO_PartialRemap2_TIM2     : TIM2 Partial2 Alternate Function mapping
  *     @arg GPIO_FullRemap_TIM2         : TIM2 Full Alternate Function mapping
  *     @arg GPIO_PartialRemap_TIM3      : TIM3 Partial Alternate Function mapping
  *     @arg GPIO_FullRemap_TIM3         : TIM3 Full Alternate Function mapping
  *     @arg GPIO_Remap_TIM4             : TIM4 Alternate Function mapping
  *     @arg GPIO_Remap1_CAN1            : CAN1 Alternate Function mapping
  *     @arg GPIO_Remap2_CAN1            : CAN1 Alternate Function mapping
  *     @arg GPIO_Remap_PD01             : PD01 Alternate Function mapping
  *     @arg GPIO_Remap_TIM5CH4_LSI      : LSI connected to TIM5 Channel4 input capture for calibration
  *     @arg GPIO_Remap_ADC1_ETRGINJ     : ADC1 External Trigger Injected Conversion remapping
  *     @arg GPIO_Remap_ADC1_ETRGREG     : ADC1 External Trigger Regular Conversion remapping
  *     @arg GPIO_Remap_ADC2_ETRGINJ     : ADC2 External Trigger Injected Conversion remapping
  *     @arg GPIO_Remap_ADC2_ETRGREG     : ADC2 External Trigger Regular Conversion remapping
  *     @arg GPIO_Remap_ETH              : Ethernet remapping (only for Connectivity line devices)
  *     @arg GPIO_Remap_CAN2             : CAN2 remapping (only for Connectivity line devices)
  *     @arg GPIO_Remap_SWJ_NoJTRST      : Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST
  *     @arg GPIO_Remap_SWJ_JTAGDisable  : JTAG-DP Disabled and SW-DP Enabled
  *     @arg GPIO_Remap_SWJ_Disable      : Full SWJ Disabled (JTAG-DP + SW-DP)
  *     @arg GPIO_Remap_SPI3             : SPI3/I2S3 Alternate Function mapping (only for Connectivity line devices)
  *                                        When the SPI3/I2S3 is remapped using this function, the SWJ is configured
  *                                        to Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST.
  *     @arg GPIO_Remap_TIM2ITR1_PTP_SOF : Ethernet PTP output or USB OTG SOF (Start of Frame) connected
  *                                        to TIM2 Internal Trigger 1 for calibration (only for Connectivity line devices)
  *                                        If the GPIO_Remap_TIM2ITR1_PTP_SOF is enabled the TIM2 ITR1 is connected to
  *                                        Ethernet PTP output. When Reset TIM2 ITR1 is connected to USB OTG SOF output.
  *     @arg GPIO_Remap_PTP_PPS          : Ethernet MAC PPS_PTS output on PB05 (only for Connectivity line devices)
  *     @arg GPIO_Remap_TIM15            : TIM15 Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM16            : TIM16 Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM17            : TIM17 Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_CEC              : CEC Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM1_DMA         : TIM1 DMA requests mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM9             : TIM9 Alternate Function mapping (only for XL-density devices)
  *     @arg GPIO_Remap_TIM10            : TIM10 Alternate Function mapping (only for XL-density devices)
  *     @arg GPIO_Remap_TIM11            : TIM11 Alternate Function mapping (only for XL-density devices)
  *     @arg GPIO_Remap_TIM13            : TIM13 Alternate Function mapping (only for High density Value line and XL-density devices)
  *     @arg GPIO_Remap_TIM14            : TIM14 Alternate Function mapping (only for High density Value line and XL-density devices)
  *     @arg GPIO_Remap_FSMC_NADV        : FSMC_NADV Alternate Function mapping (only for High density Value line and XL-density devices)
  *     @arg GPIO_Remap_TIM67_DAC_DMA    : TIM6/TIM7 and DAC DMA requests remapping (only for High density Value line devices)
  *     @arg GPIO_Remap_TIM12            : TIM12 Alternate Function mapping (only for High density Value line devices)
  *     @arg GPIO_Remap_MISC             : Miscellaneous Remap (DMA2 Channel5 Position and DAC Trigger remapping,
  *                                        only for High density Value line devices)
  * @param  NewState: new state of the port pin remapping.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void GPIO::PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState)
{
	uint32_t tmp = 0x00, tmp1 = 0x00, tmpreg = 0x00, tmpmask = 0x00;

	/* Check the parameters */
	assert_param(IS_GPIO_REMAP(GPIO_Remap));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if((GPIO_Remap & 0x80000000) == 0x80000000)
	{
		tmpreg = AFIO->MAPR2;
	}
	else
	{
		tmpreg = AFIO->MAPR;
	}

	tmpmask = (GPIO_Remap & DBGAFR_POSITION_MASK) >> 0x10;
	tmp = GPIO_Remap & LSB_MASK;

	if((GPIO_Remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK))
	{
		tmpreg &= DBGAFR_SWJCFG_MASK;
		AFIO->MAPR &= DBGAFR_SWJCFG_MASK;
	}
	else if((GPIO_Remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK)
	{
		tmp1 = ((uint32_t)0x03) << tmpmask;
		tmpreg &= ~tmp1;
		tmpreg |= ~DBGAFR_SWJCFG_MASK;
	}
	else
	{
		tmpreg &= ~(tmp << ((GPIO_Remap >> 0x15)*0x10));
		tmpreg |= ~DBGAFR_SWJCFG_MASK;
	}

	if(NewState != DISABLE)
	{
		tmpreg |= (tmp << ((GPIO_Remap >> 0x15)*0x10));
	}

	if((GPIO_Remap & 0x80000000) == 0x80000000)
	{
		AFIO->MAPR2 = tmpreg;
	}
	else
	{
		AFIO->MAPR = tmpreg;
	}
}

/**
  * @brief  Selects the GPIO pin used as EXTI Line.
  * @param  GPIO_PortSource: selects the GPIO port to be used as source for EXTI lines.
  *   This parameter can be GPIO_PortSourceGPIOx where x can be (A..G).
  * @param  GPIO_PinSource: specifies the EXTI line to be configured.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @retval None
  */
void GPIO::EXTILineConfig(uint8_t GPIO_PinSource)
{
	uint32_t tmp = 0x00;
	uint8_t GPIO_PortSource = this->GetSource();
	/* Check the parameters */
	assert_param(IS_GPIO_EXTI_PORT_SOURCE(GPIO_PortSource));
	assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));

	tmp = ((uint32_t)0x0F) << (0x04 * (GPIO_PinSource & (uint8_t)0x03));
	AFIO->EXTICR[GPIO_PinSource >> 0x02] &= ~tmp;
	AFIO->EXTICR[GPIO_PinSource >> 0x02] |= (((uint32_t)GPIO_PortSource) << (0x04 * (GPIO_PinSource & (uint8_t)0x03)));
}

/**
  * @brief  Selects the Ethernet media interface.
  * @note   This function applies only to STM32 Connectivity line devices.
  * @param  GPIO_ETH_MediaInterface: specifies the Media Interface mode.
  *   This parameter can be one of the following values:
  *     @arg GPIO_ETH_MediaInterface_MII: MII mode
  *     @arg GPIO_ETH_MediaInterface_RMII: RMII mode
  * @retval None
  */
void GPIO::ETH_MediaInterfaceConfig(uint32_t GPIO_ETH_MediaInterface)
{
	assert_param(IS_GPIO_ETH_MEDIA_INTERFACE(GPIO_ETH_MediaInterface));

	/* Configure MII_RMII selection bit */
	*(__IO uint32_t *) MAPR_MII_RMII_SEL_BB = GPIO_ETH_MediaInterface;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

GPIO_Bit::GPIO_Bit(const GPIO_Bit& Other)
{
	this->GPIOx = Other.GPIOx;
	this->Index = Other.Index;
}

bool GPIO_Bit::operator=(bool In)
{
	/*if(In) this->GPIOx->BSRR = 1<<Index;
	else this->GPIOx->BRR = 1<<Index;//寄存器操作0.458us*/
	BITBAND_REG(GPIOx->ODR, Index) = In;//位带操作0.417us
	return In;
}

GPIO_Bit& GPIO_Bit::operator=(GPIO_Bit& In)
{
	BITBAND_REG(this->GPIOx->ODR, this->Index) = BITBAND_REG(In.GPIOx->IDR, In.Index);
	return *this;
}

GPIO_Bit::operator bool()
{
	return (uint8_t)BITBAND_REG(GPIOx->IDR, Index);
}

bool GPIO_Bit::operator==(bool In)
{
	return (bool)BITBAND_REG(GPIOx->IDR, Index) == (bool)In;
}

bool GPIO_Bit::operator!()
{
	return not BITBAND_REG(GPIOx->IDR, Index);
}
bool GPIO_Bit::operator*()
{
	return BITBAND_REG(GPIOx->ODR, Index);
}
GPIO_TypeDef* GPIO_Bit::GetPeriphPointer(void)
{
	return this->GPIOx;
}

uint8_t GPIO_Bit::GetIndex(void)
{
	return this->Index;
}
