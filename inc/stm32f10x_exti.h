/**
  ******************************************************************************
  * @file    stm32f10x_exti.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the EXTI firmware
  *          library.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_EXTI_H
#define __STM32F10x_EXTI_H

//#ifdef __cplusplus
// extern "C" {
//#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @addtogroup EXTI
  * @{
  */

/** @defgroup EXTI_Exported_Types
  * @{
  */

/**
  * @brief  EXTI mode enumeration
  */

typedef enum
{
    Mode_Interrupt = 0x00,
    Mode_Event = 0x04
} EXTIMode_TypeDef;

#define IS_EXTI_MODE(MODE) (((MODE) == Mode_Interrupt) || ((MODE) == Mode_Event))

/**
  * @brief  EXTI Trigger enumeration
  */

typedef enum
{
    Trigger_Rising = 0x08,
    Trigger_Falling = 0x0C,
    Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;

#define IS_EXTI_TRIGGER(TRIGGER) (((TRIGGER) == Trigger_Rising) || \
                                  ((TRIGGER) == Trigger_Falling) || \
                                  ((TRIGGER) == Trigger_Rising_Falling))
/**
  * @brief  EXTI Init Structure definition
  */

typedef struct
{
	uint32_t Line;               /*!< Specifies the EXTI lines to be enabled or disabled.
                                         This parameter can be any combination of @ref EXTI_Lines */

	EXTIMode_TypeDef Mode;       /*!< Specifies the mode for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */

	EXTITrigger_TypeDef Trigger; /*!< Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */

	FunctionalState LineCmd;     /*!< Specifies the new state of the selected EXTI lines.
                                         This parameter can be set either to ENABLE or DISABLE */
} _EXTI_InitTypeDef;

/**
  * @}
  */

/** @defgroup EXTI_Exported_Constants
  * @{
  */

/** @defgroup EXTI_Lines
  * @{
  */

#define EXTI_Line0       ((uint32_t)0x00001)  /*!< External interrupt line 0 */
#define EXTI_Line1       ((uint32_t)0x00002)  /*!< External interrupt line 1 */
#define EXTI_Line2       ((uint32_t)0x00004)  /*!< External interrupt line 2 */
#define EXTI_Line3       ((uint32_t)0x00008)  /*!< External interrupt line 3 */
#define EXTI_Line4       ((uint32_t)0x00010)  /*!< External interrupt line 4 */
#define EXTI_Line5       ((uint32_t)0x00020)  /*!< External interrupt line 5 */
#define EXTI_Line6       ((uint32_t)0x00040)  /*!< External interrupt line 6 */
#define EXTI_Line7       ((uint32_t)0x00080)  /*!< External interrupt line 7 */
#define EXTI_Line8       ((uint32_t)0x00100)  /*!< External interrupt line 8 */
#define EXTI_Line9       ((uint32_t)0x00200)  /*!< External interrupt line 9 */
#define EXTI_Line10      ((uint32_t)0x00400)  /*!< External interrupt line 10 */
#define EXTI_Line11      ((uint32_t)0x00800)  /*!< External interrupt line 11 */
#define EXTI_Line12      ((uint32_t)0x01000)  /*!< External interrupt line 12 */
#define EXTI_Line13      ((uint32_t)0x02000)  /*!< External interrupt line 13 */
#define EXTI_Line14      ((uint32_t)0x04000)  /*!< External interrupt line 14 */
#define EXTI_Line15      ((uint32_t)0x08000)  /*!< External interrupt line 15 */
#define EXTI_Line16      ((uint32_t)0x10000)  /*!< External interrupt line 16 Connected to the PVD Output */
#define EXTI_Line17      ((uint32_t)0x20000)  /*!< External interrupt line 17 Connected to the RTC Alarm event */
#define EXTI_Line18      ((uint32_t)0x40000)  /*!< External interrupt line 18 Connected to the USB Device/USB OTG FS
                                                   Wakeup from suspend event */
#define EXTI_Line19      ((uint32_t)0x80000)  /*!< External interrupt line 19 Connected to the Ethernet Wakeup event */

#define IS_EXTI_LINE(LINE) ((((LINE) & (uint32_t)0xFFF00000) == 0x00) && ((LINE) != (uint16_t)0x00))
#define IS_GET_EXTI_LINE(LINE) (((LINE) == EXTI_Line0) || ((LINE) == EXTI_Line1) || \
                                ((LINE) == EXTI_Line2) || ((LINE) == EXTI_Line3) || \
                                ((LINE) == EXTI_Line4) || ((LINE) == EXTI_Line5) || \
                                ((LINE) == EXTI_Line6) || ((LINE) == EXTI_Line7) || \
                                ((LINE) == EXTI_Line8) || ((LINE) == EXTI_Line9) || \
                                ((LINE) == EXTI_Line10) || ((LINE) == EXTI_Line11) || \
                                ((LINE) == EXTI_Line12) || ((LINE) == EXTI_Line13) || \
                                ((LINE) == EXTI_Line14) || ((LINE) == EXTI_Line15) || \
                                ((LINE) == EXTI_Line16) || ((LINE) == EXTI_Line17) || \
                                ((LINE) == EXTI_Line18) || ((LINE) == EXTI_Line19))


/**
  * @}
  */

/**
  * @}
  */

/** @defgroup EXTI_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup EXTI_Exported_Functions
  * @{
  */
class EXTI
{
	private:
		uint32_t Line;
	public:
		typedef _EXTI_InitTypeDef InitTypeDef;
		EXTI(uint32_t _Line):Line(1<<_Line) {};
		EXTI(uint32_t _Line, InitTypeDef& InitStruct);
		EXTI(uint32_t _Line, EXTITrigger_TypeDef Trigger\
		     , EXTIMode_TypeDef Mode = Mode_Interrupt, FunctionalState LineCmd = ENABLE);
		EXTI(const EXTI& Other);

		EXTI_TypeDef* GetPeriphPointer(void);//获取内部外设指针
		void DeInit(void);
		void Init(InitTypeDef &InitStruct);
		void StructInit(InitTypeDef &InitStruct);
		void GenerateSWInterrupt(void);
		FlagStatus GetFlagStatus(void);
		void ClearFlag(void);
		ITStatus GetITStatus(void);
		void ClearITPendingBit(void);
};
//#ifdef __cplusplus
//}
//#endif
#endif /* __STM32F10x_EXTI_H */
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
