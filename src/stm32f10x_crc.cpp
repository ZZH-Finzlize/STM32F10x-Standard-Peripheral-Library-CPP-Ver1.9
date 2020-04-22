/**
  ******************************************************************************
  * @file    stm32f10x_crc.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file provides all the CRC firmware functions.
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
#include "stm32f10x_crc.h"
/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup CRC
  * @brief CRC driver modules
  * @{
  */

/** @defgroup CRC_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup CRC_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup CRC_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup CRC_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup CRC_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup CRC_Private_Functions
  * @{
  */

CRC::CRC(uint8_t IDValue)
{
	this->SetIDRegister(IDValue);
}

CRC_TypeDef* CRC::GetPeriphPointer(void)
{
	return _CRC;
}

CRC& CRC::operator>>(uint32_t& Res)
{
	Res = this->GetCRC();    //提取CRC对象即读取DR寄存器
	return *this;
}

uint32_t CRC::operator<<(uint32_t Data)
{
	return this->CalcCRC(Data);   //不支持连续插入来获得多个CRC
}

void CRC::operator=(uint32_t Data)
{
	_CRC->DR = Data;   //向CRC对象赋值即设置DR寄存器
}

/**
  * @brief  Resets the CRC Data register (DR).
  * @param  None
  * @retval None
  */
void CRC::ResetDR(void)
{
	/* Reset CRC generator */
	_CRC->CR = CRC_CR_RESET;
}

/**
  * @brief  Computes the 32-bit CRC of a given data word(32-bit).
  * @param  Data: data word(32-bit) to compute its CRC
  * @retval 32-bit CRC
  */
uint32_t CRC::CalcCRC(uint32_t Data)
{
	_CRC->DR = Data;

	return (_CRC->DR);
}

/**
  * @brief  Computes the 32-bit CRC of a given buffer of data word(32-bit).
  * @param  pBuffer: pointer to the buffer containing the data to be computed
  * @param  BufferLength: length of the buffer to be computed
  * @retval 32-bit CRC
  */
uint32_t CRC::CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength)
{
	uint32_t index = 0;

	for(index = 0; index < BufferLength; index++)
	{
		_CRC->DR = pBuffer[index];
	}
	return (_CRC->DR);
}

/**
  * @brief  Returns the current CRC value.
  * @param  None
  * @retval 32-bit CRC
  */
uint32_t  CRC::GetCRC(void)
{
	return (_CRC->DR);
}

/**
  * @brief  Stores a 8-bit data in the Independent Data(ID) register.
  * @param  IDValue: 8-bit value to be stored in the ID register
  * @retval None
  */
void  CRC::SetIDRegister(uint8_t IDValue)
{
	_CRC->IDR = IDValue;
}

/**
  * @brief  Returns the 8-bit data stored in the Independent Data(ID) register
  * @param  None
  * @retval 8-bit value of the ID register
  */
uint8_t CRC::GetIDRegister(void)
{
	return (_CRC->IDR);
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
