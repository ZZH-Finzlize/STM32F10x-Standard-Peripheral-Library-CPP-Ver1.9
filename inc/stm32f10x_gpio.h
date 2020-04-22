/**
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the GPIO
  *          firmware library.
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
#ifndef __STM32F10x_GPIO_H
#define __STM32F10x_GPIO_H

//#ifdef __cplusplus
// extern "C" {
//#endif
#include "stm32f10x.h"
extern "C++"
{
	/* Includes ------------------------------------------------------------------*/

	/** @addtogroup STM32F10x_StdPeriph_Driver
	  * @{
	  */

	/** @addtogroup GPIO
	  * @{
	  */

	/** @defgroup GPIO_Exported_Types
	  * @{
	  */

#define IS_GPIO_ALL_PERIPH(PERIPH) (((PERIPH) == GPIOA) || \
                                    ((PERIPH) == GPIOB) || \
                                    ((PERIPH) == GPIOC) || \
                                    ((PERIPH) == GPIOD) || \
                                    ((PERIPH) == GPIOE) || \
                                    ((PERIPH) == GPIOF) || \
                                    ((PERIPH) == GPIOG))

	/**
	  * @brief  Output Maximum frequency selection
	  */

	typedef enum
	{
		Speed_10MHz = 1,
		Speed_2MHz,
		Speed_50MHz
	}
	GPIOSpeed_TypeDef;
#define IS_GPIO_SPEED(SPEED) (((SPEED) == Speed_10MHz) || ((SPEED) == Speed_2MHz) || \
                              ((SPEED) == Speed_50MHz))

	/**
	  * @brief  Configuration Mode enumeration
	  */

	typedef enum
	{
	    Mode_AIN = 0x0,//模拟输入
	    Mode_IN_FLOATING = 0x04,//浮空输入
	    Mode_IPD = 0x28,//下拉输入
	    Mode_IPU = 0x48,//上拉输入
	    Mode_Out_OD = 0x14,//开漏输出
	    Mode_Out_PP = 0x10,//推挽输出
	    Mode_AF_OD = 0x1C,//复用开漏输出
	    Mode_AF_PP = 0x18//复用推挽输出
	} GPIOMode_TypeDef;

#define IS_GPIO_MODE(MODE) (((MODE) == Mode_AIN) || ((MODE) == Mode_IN_FLOATING) || \
                            ((MODE) == Mode_IPD) || ((MODE) == Mode_IPU) || \
                            ((MODE) == Mode_Out_OD) || ((MODE) == Mode_Out_PP) || \
                            ((MODE) == Mode_AF_OD) || ((MODE) == Mode_AF_PP))

	/**
	  * @brief  GPIO Init structure definition
	  */

	typedef struct
	{
		uint16_t Pin;             /*!< Specifies the GPIO pins to be configured.
                                      This parameter can be any value of @ref GPIO_pins_define */

		GPIOSpeed_TypeDef Speed;  /*!< Specifies the speed for the selected pins.
                                      This parameter can be a value of @ref GPIOSpeed_TypeDef */

		GPIOMode_TypeDef Mode;    /*!< Specifies the operating mode for the selected pins.
                                      This parameter can be a value of @ref GPIOMode_TypeDef */
	} _GPIO_InitTypeDef;


	/**
	  * @brief  Bit_SET and Bit_RESET enumeration
	  */

	typedef enum
	{
	    Bit_RESET = 0,
	    Bit_SET
	} BitAction;

#define IS_GPIO_BIT_ACTION(ACTION) (((ACTION) == Bit_RESET) || ((ACTION) == Bit_SET))

	/**
	  * @}
	  */

	/** @defgroup GPIO_Exported_Constants
	  * @{
	  */

	/** @defgroup GPIO_pins_define
	  * @{
	  */

#define Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected */
#define Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected */
#define Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected */
#define Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
#define Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */
#define Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected */
#define Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected */
#define Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected */
#define Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected */
#define Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected */
#define Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected */
#define Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected */
#define Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected */
#define Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected */
#define Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected */
#define Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected */
#define Pin_All               		 ((uint16_t)0xFFFF)  /*!< All pins selected */

#define IS_GPIO_PIN(PIN) ((((PIN) & (uint16_t)0x00) == 0x00) && ((PIN) != (uint16_t)0x00))

#define IS_GET_GPIO_PIN(PIN) (((PIN) == Pin_0) || \
                              ((PIN) == Pin_1) || \
                              ((PIN) == Pin_2) || \
                              ((PIN) == Pin_3) || \
                              ((PIN) == Pin_4) || \
                              ((PIN) == Pin_5) || \
                              ((PIN) == Pin_6) || \
                              ((PIN) == Pin_7) || \
                              ((PIN) == Pin_8) || \
                              ((PIN) == Pin_9) || \
                              ((PIN) == Pin_10) || \
                              ((PIN) == Pin_11) || \
                              ((PIN) == Pin_12) || \
                              ((PIN) == Pin_13) || \
                              ((PIN) == Pin_14) || \
                              ((PIN) == Pin_15))

	/**
	  * @}
	  */

	/** @defgroup GPIO_Remap_define
	  * @{
	  */

#define GPIO_Remap_SPI1             ((uint32_t)0x00000001)  /*!< SPI1 Alternate Function mapping */
#define GPIO_Remap_I2C1             ((uint32_t)0x00000002)  /*!< I2C1 Alternate Function mapping */
#define GPIO_Remap_USART1           ((uint32_t)0x00000004)  /*!< USART1 Alternate Function mapping */
#define GPIO_Remap_USART2           ((uint32_t)0x00000008)  /*!< USART2 Alternate Function mapping */
#define GPIO_PartialRemap_USART3    ((uint32_t)0x00140010)  /*!< USART3 Partial Alternate Function mapping */
#define GPIO_FullRemap_USART3       ((uint32_t)0x00140030)  /*!< USART3 Full Alternate Function mapping */
#define GPIO_PartialRemap_TIM1      ((uint32_t)0x00160040)  /*!< TIM1 Partial Alternate Function mapping */
#define GPIO_FullRemap_TIM1         ((uint32_t)0x001600C0)  /*!< TIM1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM2     ((uint32_t)0x00180100)  /*!< TIM2 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM2     ((uint32_t)0x00180200)  /*!< TIM2 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_TIM2         ((uint32_t)0x00180300)  /*!< TIM2 Full Alternate Function mapping */
#define GPIO_PartialRemap_TIM3      ((uint32_t)0x001A0800)  /*!< TIM3 Partial Alternate Function mapping */
#define GPIO_FullRemap_TIM3         ((uint32_t)0x001A0C00)  /*!< TIM3 Full Alternate Function mapping */
#define GPIO_Remap_TIM4             ((uint32_t)0x00001000)  /*!< TIM4 Alternate Function mapping */
#define GPIO_Remap1_CAN1            ((uint32_t)0x001D4000)  /*!< CAN1 Alternate Function mapping */
#define GPIO_Remap2_CAN1            ((uint32_t)0x001D6000)  /*!< CAN1 Alternate Function mapping */
#define GPIO_Remap_PD01             ((uint32_t)0x00008000)  /*!< PD01 Alternate Function mapping */
#define GPIO_Remap_TIM5CH4_LSI      ((uint32_t)0x00200001)  /*!< LSI connected to TIM5 Channel4 input capture for calibration */
#define GPIO_Remap_ADC1_ETRGINJ     ((uint32_t)0x00200002)  /*!< ADC1 External Trigger Injected Conversion remapping */
#define GPIO_Remap_ADC1_ETRGREG     ((uint32_t)0x00200004)  /*!< ADC1 External Trigger Regular Conversion remapping */
#define GPIO_Remap_ADC2_ETRGINJ     ((uint32_t)0x00200008)  /*!< ADC2 External Trigger Injected Conversion remapping */
#define GPIO_Remap_ADC2_ETRGREG     ((uint32_t)0x00200010)  /*!< ADC2 External Trigger Regular Conversion remapping */
#define GPIO_Remap_ETH              ((uint32_t)0x00200020)  /*!< Ethernet remapping (only for Connectivity line devices) */
#define GPIO_Remap_CAN2             ((uint32_t)0x00200040)  /*!< CAN2 remapping (only for Connectivity line devices) */
#define GPIO_Remap_SWJ_NoJTRST      ((uint32_t)0x00300100)  /*!< Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST */
#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200)  /*!< JTAG-DP Disabled and SW-DP Enabled */
#define GPIO_Remap_SWJ_Disable      ((uint32_t)0x00300400)  /*!< Full SWJ Disabled (JTAG-DP + SW-DP) */
#define GPIO_Remap_SPI3             ((uint32_t)0x00201100)  /*!< SPI3/I2S3 Alternate Function mapping (only for Connectivity line devices) */
#define GPIO_Remap_TIM2ITR1_PTP_SOF ((uint32_t)0x00202000)  /*!< Ethernet PTP output or USB OTG SOF (Start of Frame) connected
                                                                 to TIM2 Internal Trigger 1 for calibration
                                                                 (only for Connectivity line devices) */
#define GPIO_Remap_PTP_PPS          ((uint32_t)0x00204000)  /*!< Ethernet MAC PPS_PTS output on PB05 (only for Connectivity line devices) */

#define GPIO_Remap_TIM15            ((uint32_t)0x80000001)  /*!< TIM15 Alternate Function mapping (only for Value line devices) */
#define GPIO_Remap_TIM16            ((uint32_t)0x80000002)  /*!< TIM16 Alternate Function mapping (only for Value line devices) */
#define GPIO_Remap_TIM17            ((uint32_t)0x80000004)  /*!< TIM17 Alternate Function mapping (only for Value line devices) */
#define GPIO_Remap_CEC              ((uint32_t)0x80000008)  /*!< CEC Alternate Function mapping (only for Value line devices) */
#define GPIO_Remap_TIM1_DMA         ((uint32_t)0x80000010)  /*!< TIM1 DMA requests mapping (only for Value line devices) */

#define GPIO_Remap_TIM9             ((uint32_t)0x80000020)  /*!< TIM9 Alternate Function mapping (only for XL-density devices) */
#define GPIO_Remap_TIM10            ((uint32_t)0x80000040)  /*!< TIM10 Alternate Function mapping (only for XL-density devices) */
#define GPIO_Remap_TIM11            ((uint32_t)0x80000080)  /*!< TIM11 Alternate Function mapping (only for XL-density devices) */
#define GPIO_Remap_TIM13            ((uint32_t)0x80000100)  /*!< TIM13 Alternate Function mapping (only for High density Value line and XL-density devices) */
#define GPIO_Remap_TIM14            ((uint32_t)0x80000200)  /*!< TIM14 Alternate Function mapping (only for High density Value line and XL-density devices) */
#define GPIO_Remap_FSMC_NADV        ((uint32_t)0x80000400)  /*!< FSMC_NADV Alternate Function mapping (only for High density Value line and XL-density devices) */

#define GPIO_Remap_TIM67_DAC_DMA    ((uint32_t)0x80000800)  /*!< TIM6/TIM7 and DAC DMA requests remapping (only for High density Value line devices) */
#define GPIO_Remap_TIM12            ((uint32_t)0x80001000)  /*!< TIM12 Alternate Function mapping (only for High density Value line devices) */
#define GPIO_Remap_MISC             ((uint32_t)0x80002000)  /*!< Miscellaneous Remap (DMA2 Channel5 Position and DAC Trigger remapping, 
                                                                 only for High density Value line devices) */

#define IS_GPIO_REMAP(REMAP) (((REMAP) == GPIO_Remap_SPI1) || ((REMAP) == GPIO_Remap_I2C1) || \
                              ((REMAP) == GPIO_Remap_USART1) || ((REMAP) == GPIO_Remap_USART2) || \
                              ((REMAP) == GPIO_PartialRemap_USART3) || ((REMAP) == GPIO_FullRemap_USART3) || \
                              ((REMAP) == GPIO_PartialRemap_TIM1) || ((REMAP) == GPIO_FullRemap_TIM1) || \
                              ((REMAP) == GPIO_PartialRemap1_TIM2) || ((REMAP) == GPIO_PartialRemap2_TIM2) || \
                              ((REMAP) == GPIO_FullRemap_TIM2) || ((REMAP) == GPIO_PartialRemap_TIM3) || \
                              ((REMAP) == GPIO_FullRemap_TIM3) || ((REMAP) == GPIO_Remap_TIM4) || \
                              ((REMAP) == GPIO_Remap1_CAN1) || ((REMAP) == GPIO_Remap2_CAN1) || \
                              ((REMAP) == GPIO_Remap_PD01) || ((REMAP) == GPIO_Remap_TIM5CH4_LSI) || \
                              ((REMAP) == GPIO_Remap_ADC1_ETRGINJ) ||((REMAP) == GPIO_Remap_ADC1_ETRGREG) || \
                              ((REMAP) == GPIO_Remap_ADC2_ETRGINJ) ||((REMAP) == GPIO_Remap_ADC2_ETRGREG) || \
                              ((REMAP) == GPIO_Remap_ETH) ||((REMAP) == GPIO_Remap_CAN2) || \
                              ((REMAP) == GPIO_Remap_SWJ_NoJTRST) || ((REMAP) == GPIO_Remap_SWJ_JTAGDisable) || \
                              ((REMAP) == GPIO_Remap_SWJ_Disable)|| ((REMAP) == GPIO_Remap_SPI3) || \
                              ((REMAP) == GPIO_Remap_TIM2ITR1_PTP_SOF) || ((REMAP) == GPIO_Remap_PTP_PPS) || \
                              ((REMAP) == GPIO_Remap_TIM15) || ((REMAP) == GPIO_Remap_TIM16) || \
                              ((REMAP) == GPIO_Remap_TIM17) || ((REMAP) == GPIO_Remap_CEC) || \
                              ((REMAP) == GPIO_Remap_TIM1_DMA) || ((REMAP) == GPIO_Remap_TIM9) || \
                              ((REMAP) == GPIO_Remap_TIM10) || ((REMAP) == GPIO_Remap_TIM11) || \
                              ((REMAP) == GPIO_Remap_TIM13) || ((REMAP) == GPIO_Remap_TIM14) || \
                              ((REMAP) == GPIO_Remap_FSMC_NADV) || ((REMAP) == GPIO_Remap_TIM67_DAC_DMA) || \
                              ((REMAP) == GPIO_Remap_TIM12) || ((REMAP) == GPIO_Remap_MISC))

	/**
	  * @}
	  */

	/** @defgroup GPIO_Port_Sources
	  * @{
	  */

#define GPIO_PortSourceGPIOA       ((uint8_t)0x00)
#define GPIO_PortSourceGPIOB       ((uint8_t)0x01)
#define GPIO_PortSourceGPIOC       ((uint8_t)0x02)
#define GPIO_PortSourceGPIOD       ((uint8_t)0x03)
#define GPIO_PortSourceGPIOE       ((uint8_t)0x04)
#define GPIO_PortSourceGPIOF       ((uint8_t)0x05)
#define GPIO_PortSourceGPIOG       ((uint8_t)0x06)
#define IS_GPIO_EVENTOUT_PORT_SOURCE(PORTSOURCE) (((PORTSOURCE) == GPIO_PortSourceGPIOA) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOB) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOC) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOD) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOE))

#define IS_GPIO_EXTI_PORT_SOURCE(PORTSOURCE) (((PORTSOURCE) == GPIO_PortSourceGPIOA) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOB) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOC) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOD) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOE) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOF) || \
        ((PORTSOURCE) == GPIO_PortSourceGPIOG))

	/**
	  * @}
	  */

	/** @defgroup GPIO_Pin_sources
	  * @{
	  */

#define GPIO_PinSource0            ((uint8_t)0x00)
#define GPIO_PinSource1            ((uint8_t)0x01)
#define GPIO_PinSource2            ((uint8_t)0x02)
#define GPIO_PinSource3            ((uint8_t)0x03)
#define GPIO_PinSource4            ((uint8_t)0x04)
#define GPIO_PinSource5            ((uint8_t)0x05)
#define GPIO_PinSource6            ((uint8_t)0x06)
#define GPIO_PinSource7            ((uint8_t)0x07)
#define GPIO_PinSource8            ((uint8_t)0x08)
#define GPIO_PinSource9            ((uint8_t)0x09)
#define GPIO_PinSource10           ((uint8_t)0x0A)
#define GPIO_PinSource11           ((uint8_t)0x0B)
#define GPIO_PinSource12           ((uint8_t)0x0C)
#define GPIO_PinSource13           ((uint8_t)0x0D)
#define GPIO_PinSource14           ((uint8_t)0x0E)
#define GPIO_PinSource15           ((uint8_t)0x0F)

#define IS_GPIO_PIN_SOURCE(PINSOURCE) (((PINSOURCE) == GPIO_PinSource0) || \
                                       ((PINSOURCE) == GPIO_PinSource1) || \
                                       ((PINSOURCE) == GPIO_PinSource2) || \
                                       ((PINSOURCE) == GPIO_PinSource3) || \
                                       ((PINSOURCE) == GPIO_PinSource4) || \
                                       ((PINSOURCE) == GPIO_PinSource5) || \
                                       ((PINSOURCE) == GPIO_PinSource6) || \
                                       ((PINSOURCE) == GPIO_PinSource7) || \
                                       ((PINSOURCE) == GPIO_PinSource8) || \
                                       ((PINSOURCE) == GPIO_PinSource9) || \
                                       ((PINSOURCE) == GPIO_PinSource10) || \
                                       ((PINSOURCE) == GPIO_PinSource11) || \
                                       ((PINSOURCE) == GPIO_PinSource12) || \
                                       ((PINSOURCE) == GPIO_PinSource13) || \
                                       ((PINSOURCE) == GPIO_PinSource14) || \
                                       ((PINSOURCE) == GPIO_PinSource15))

	/**
	  * @}
	  */

	/** @defgroup Ethernet_Media_Interface
	  * @{
	  */
#define GPIO_ETH_MediaInterface_MII    ((u32)0x00000000)
#define GPIO_ETH_MediaInterface_RMII   ((u32)0x00000001)

#define IS_GPIO_ETH_MEDIA_INTERFACE(INTERFACE) (((INTERFACE) == GPIO_ETH_MediaInterface_MII) || \
        ((INTERFACE) == GPIO_ETH_MediaInterface_RMII))

	/**
	  * @}
	  */
	/**
	  * @}
	  */

	/** @defgroup GPIO_Exported_Macros
	  * @{
	  */

	/**
	  * @}
	  */

	/** @defgroup GPIO_Exported_Functions
	  * @{
	  */
#define BITBAND_REG(Reg,Bit) \
	(*((uint32_t volatile*) \
	   (0x42000000u + (((uint32_t)&(Reg) - (uint32_t)0x40000000u)<<5) + (((uint32_t)(Bit))<<2))))

	class GPIO_Bit
	{
		private:
			GPIO_TypeDef* GPIOx;
			uint8_t Index;
		public:
			friend class GPIO;
			GPIO_Bit(GPIO_TypeDef* _GPIOx, uint8_t _Index) :GPIOx(_GPIOx), Index(_Index%16) {};
			GPIO_Bit(const GPIO_Bit& Other);
			bool operator = (bool In);//左值重载
			GPIO_Bit& operator = (GPIO_Bit& In);//左值重载
			operator bool();//右值重载
			bool operator == (bool In);//逻辑比较重载
			bool operator !(); //逻辑非重载
			bool operator *();//读取ODR
			GPIO_TypeDef* GetPeriphPointer(void);
			uint8_t GetIndex(void);
	};
	class GPIO
	{
		private:
			GPIO_TypeDef* GPIOx;
			static uint16_t ObjectNum[7];
			void Increase(void);
			uint16_t Decrease(void);
			uint8_t GetSource(void);
		public:
			friend class RCC;
			typedef GPIO_Bit Bit;
			typedef _GPIO_InitTypeDef InitTypeDef;

			GPIO(GPIO_TypeDef* _GPIOx):GPIOx(_GPIOx)
			{
				this->Increase();
			}
			GPIO(GPIO_TypeDef* _GPIOx,InitTypeDef &InitStruct);
			GPIO(const GPIO& Other);

			void operator <<= (uint16_t Data);
			void operator >>= (uint16_t Data);

			GPIO& operator = (uint16_t Data);
			operator uint16_t();
			bool operator == (uint16_t Data);

			uint16_t operator * (void);
			uint16_t operator ~(void);
			GPIO_Bit operator [](uint8_t Pin); //位操作重载
			GPIO_TypeDef* GetPeriphPointer(void);
			~GPIO();



			void DeInit(void);
			void AFIODeInit(void);
			void Init(InitTypeDef &InitStruct);
			void StructInit(InitTypeDef &InitStruct);
			bool ReadInputDataBit(uint16_t Pin);
			uint16_t ReadInputData(void);
			bool ReadOutputDataBit(uint16_t Pin);
			uint16_t ReadOutputData(void);
			void SetBits(uint16_t Pin);
			void ResetBits(uint16_t Pin);
			void WriteBit(uint16_t Pin, bool BitVal);
			void Write(uint16_t PortVal);
			void PinLockConfig(uint16_t Pin);
			void EventOutputConfig(uint8_t GPIO_PinSource);
			void EventOutputCmd(FunctionalState NewState);
			void PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
			void EXTILineConfig(uint8_t GPIO_PinSource);
			void ETH_MediaInterfaceConfig(uint32_t GPIO_ETH_MediaInterface);
	};
//#ifdef __cplusplus
//}
//#endif
}
#endif /* __STM32F10x_GPIO_H */
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