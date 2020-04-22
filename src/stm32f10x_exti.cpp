/**
  ******************************************************************************
  * @file    stm32f10x_exti.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file provides all the EXTI firmware functions.
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
#include "stm32f10x_exti.h"

/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup EXTI
  * @brief EXTI driver modules
  * @{
  */

/** @defgroup EXTI_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup EXTI_Private_Defines
  * @{
  */

#define EXTI_LINENONE    ((uint32_t)0x00000)  /* No interrupt selected */

/**
  * @}
  */

/** @defgroup EXTI_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup EXTI_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup EXTI_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup EXTI_Private_Functions
  * @{
  */

EXTI_TypeDef* EXTI::GetPeriphPointer(void)
{
	return _EXTI;
}

/**
  * @brief  Deinitializes the EXTI peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
void EXTI::DeInit(void)
{
	_EXTI->IMR = 0x00000000;
	_EXTI->EMR = 0x00000000;
	_EXTI->RTSR = 0x00000000;
	_EXTI->FTSR = 0x00000000;
	_EXTI->PR = 0x000FFFFF;
}

/**
  * @brief  Initializes the EXTI peripheral according to the specified
  *         parameters in the EXTI_InitStruct.
  * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure
  *         that contains the configuration information for the EXTI peripheral.
  * @retval None
  */
void EXTI::Init(InitTypeDef &InitStruct)
{
	uint32_t tmp = 0;

	/* Check the parameters */
	assert_param(IS_EXTI_MODE(InitStruct.Mode));
	assert_param(IS_EXTI_TRIGGER(InitStruct.Trigger));
	assert_param(IS_EXTI_LINE(InitStruct.Line));
	assert_param(IS_FUNCTIONAL_STATE(InitStruct.LineCmd));

	tmp = (uint32_t)EXTI_BASE;

	if(InitStruct.LineCmd != DISABLE)
	{
		/* Clear EXTI line configuration */
		_EXTI->IMR &= ~InitStruct.Line;
		_EXTI->EMR &= ~InitStruct.Line;

		tmp += InitStruct.Mode;

		*(__IO uint32_t *) tmp |= InitStruct.Line;

		/* Clear Rising Falling edge configuration */
		_EXTI->RTSR &= ~InitStruct.Line;
		_EXTI->FTSR &= ~InitStruct.Line;

		/* Select the trigger for the selected external interrupts */
		if(InitStruct.Trigger == Trigger_Rising_Falling)
		{
			/* Rising Falling edge */
			_EXTI->RTSR |= InitStruct.Line;
			_EXTI->FTSR |= InitStruct.Line;
		}
		else
		{
			tmp = (uint32_t)EXTI_BASE;
			tmp += InitStruct.Trigger;

			*(__IO uint32_t *) tmp |= InitStruct.Line;
		}
	}
	else
	{
		tmp += InitStruct.Mode;

		/* Disable the selected external lines */
		*(__IO uint32_t *) tmp &= ~InitStruct.Line;
	}
}

/**
  * @brief  Fills each EXTI_InitStruct member with its reset value.
  * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void EXTI::StructInit(InitTypeDef &InitStruct)
{
	InitStruct.Line = EXTI_LINENONE;
	InitStruct.Mode = Mode_Interrupt;
	InitStruct.Trigger = Trigger_Falling;
	InitStruct.LineCmd = DISABLE;
}

/**
  * @brief  Generates a Software interrupt.
  * @param  Line: specifies the EXTI lines to be enabled or disabled.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..19).
  * @retval None
  */
void EXTI::GenerateSWInterrupt(void)
{
	/* Check the parameters */
	assert_param(IS_EXTI_LINE(Line));

	_EXTI->SWIER |= Line;
}

/**
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  Line: specifies the EXTI line flag to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The new state of Line (SET or RESET).
  */
FlagStatus EXTI::GetFlagStatus(void)
{
	FlagStatus bitstatus = RESET;
	/* Check the parameters */
	assert_param(IS_GET_EXTI_LINE(Line));

	if((_EXTI->PR & Line) != (uint32_t)RESET)
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
  * @brief  Clears the EXTI's line pending flags.
  * @param  Line: specifies the EXTI lines flags to clear.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..19).
  * @retval None
  */
void EXTI::ClearFlag(void)
{
	/* Check the parameters */
	assert_param(IS_EXTI_LINE(Line));

	_EXTI->PR = Line;
}

/**
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The new state of Line (SET or RESET).
  */
ITStatus EXTI::GetITStatus(void)
{
	ITStatus bitstatus = RESET;
	uint32_t enablestatus = 0;
	/* Check the parameters */
	assert_param(IS_GET_EXTI_LINE(Line));

	enablestatus =  _EXTI->IMR & Line;
	if(((_EXTI->PR & Line) != (uint32_t)RESET) && (enablestatus != (uint32_t)RESET))
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
  * @brief  Clears the EXTI's line pending bits.
  * @param  Line: specifies the EXTI lines to clear.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..19).
  * @retval None
  */
void EXTI::ClearITPendingBit(void)
{
	/* Check the parameters */
	assert_param(IS_EXTI_LINE(Line));

	_EXTI->PR = Line;
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

EXTI::EXTI(uint32_t _Line, InitTypeDef& InitStruct) :Line(1 << _Line)
{
	this->Init(InitStruct);
}

EXTI::EXTI(uint32_t _Line, EXTITrigger_TypeDef Trigger, EXTIMode_TypeDef Mode, FunctionalState LineCmd) : Line(1 << _Line)
{
	EXTI::InitTypeDef aaa;
	aaa.Line = Line;
	aaa.Mode = Mode;
	aaa.Trigger = Trigger;
	aaa.LineCmd = LineCmd;
	this->Init(aaa);
}

EXTI::EXTI(const EXTI& Other)
{
	this->Line = Other.Line;
}
