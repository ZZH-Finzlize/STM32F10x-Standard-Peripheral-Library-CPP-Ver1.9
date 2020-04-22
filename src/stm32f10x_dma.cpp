/**
  ******************************************************************************
  * @file    stm32f10x_dma.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file provides all the DMA firmware functions.
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
#include "stm32f10x_dma.h"
uint16_t DMA::ObjectNum[2] = {0,0};

/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup DMA
  * @brief DMA driver modules
  * @{
  */

/** @defgroup DMA_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup DMA_Private_Defines
  * @{
  */


/* DMA1 Channelx interrupt pending bit masks */
#define DMA1_Channel1_IT_Mask    ((uint32_t)(DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1))
#define DMA1_Channel2_IT_Mask    ((uint32_t)(DMA_ISR_GIF2 | DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_TEIF2))
#define DMA1_Channel3_IT_Mask    ((uint32_t)(DMA_ISR_GIF3 | DMA_ISR_TCIF3 | DMA_ISR_HTIF3 | DMA_ISR_TEIF3))
#define DMA1_Channel4_IT_Mask    ((uint32_t)(DMA_ISR_GIF4 | DMA_ISR_TCIF4 | DMA_ISR_HTIF4 | DMA_ISR_TEIF4))
#define DMA1_Channel5_IT_Mask    ((uint32_t)(DMA_ISR_GIF5 | DMA_ISR_TCIF5 | DMA_ISR_HTIF5 | DMA_ISR_TEIF5))
#define DMA1_Channel6_IT_Mask    ((uint32_t)(DMA_ISR_GIF6 | DMA_ISR_TCIF6 | DMA_ISR_HTIF6 | DMA_ISR_TEIF6))
#define DMA1_Channel7_IT_Mask    ((uint32_t)(DMA_ISR_GIF7 | DMA_ISR_TCIF7 | DMA_ISR_HTIF7 | DMA_ISR_TEIF7))

/* DMA2 Channelx interrupt pending bit masks */
#define DMA2_Channel1_IT_Mask    ((uint32_t)(DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1))
#define DMA2_Channel2_IT_Mask    ((uint32_t)(DMA_ISR_GIF2 | DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_TEIF2))
#define DMA2_Channel3_IT_Mask    ((uint32_t)(DMA_ISR_GIF3 | DMA_ISR_TCIF3 | DMA_ISR_HTIF3 | DMA_ISR_TEIF3))
#define DMA2_Channel4_IT_Mask    ((uint32_t)(DMA_ISR_GIF4 | DMA_ISR_TCIF4 | DMA_ISR_HTIF4 | DMA_ISR_TEIF4))
#define DMA2_Channel5_IT_Mask    ((uint32_t)(DMA_ISR_GIF5 | DMA_ISR_TCIF5 | DMA_ISR_HTIF5 | DMA_ISR_TEIF5))

/* DMA2 FLAG mask */
#define FLAG_Mask                ((uint32_t)0x10000000)

/* DMA registers Masks */
#define CCR_CLEAR_Mask           ((uint32_t)0xFFFF800F)

/**
  * @}
  */

/** @defgroup DMA_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup DMA_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup DMA_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

void DMA::Increase(void)
{
	switch((uint32_t)DMAy_Channelx)
	{
		case DMA1_Channel1_BASE:
		case DMA1_Channel2_BASE:
		case DMA1_Channel3_BASE:
		case DMA1_Channel4_BASE:
		case DMA1_Channel5_BASE:
		case DMA1_Channel6_BASE:
		case DMA1_Channel7_BASE:
			ObjectNum[0]++;
			break;
		case DMA2_Channel1_BASE:
		case DMA2_Channel2_BASE:
		case DMA2_Channel3_BASE:
		case DMA2_Channel4_BASE:
		case DMA2_Channel5_BASE:
			ObjectNum[1]++;
			break;
	}
}

uint16_t DMA::Decrease(void)
{
	switch((uint32_t)DMAy_Channelx)
	{
		case DMA1_Channel1_BASE:
		case DMA1_Channel2_BASE:
		case DMA1_Channel3_BASE:
		case DMA1_Channel4_BASE:
		case DMA1_Channel5_BASE:
		case DMA1_Channel6_BASE:
		case DMA1_Channel7_BASE:
			return --ObjectNum[0];
		case DMA2_Channel1_BASE:
		case DMA2_Channel2_BASE:
		case DMA2_Channel3_BASE:
		case DMA2_Channel4_BASE:
		case DMA2_Channel5_BASE:
			return --ObjectNum[1];
	}
	return 0xFFFF;
}

DMA::DMA(InitTypeDef& InitStruct)
{
	this->Increase();
	this->Init(InitStruct);
	this->On();
}

DMA::DMA(DMA_Channel_TypeDef* _DMAy_Channelx, InitTypeDef& InitStruct) :DMAy_Channelx(_DMAy_Channelx)
{
	this->Increase();
	this->Init(InitStruct);
	this->On();
}

DMA::DMA(DMA_Channel_TypeDef* _DMAy_Channelx, uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, uint32_t DIR, uint32_t BufferSize, uint32_t PeripheralInc, uint32_t MemoryInc, uint32_t PeripheralDataSize, uint32_t MemoryDataSize, uint32_t Mode, uint32_t Priority, uint32_t M2M) :DMAy_Channelx(_DMAy_Channelx)
{
	DMA::InitTypeDef aaa;
	this->Increase();
	aaa.PeripheralBaseAddr = PeripheralBaseAddr;
	aaa.MemoryBaseAddr = MemoryBaseAddr;
	aaa.DIR = DIR;
	aaa.BufferSize = BufferSize;
	aaa.PeripheralInc = PeripheralInc;
	aaa.MemoryInc = MemoryInc;
	aaa.PeripheralDataSize = PeripheralDataSize;
	aaa.MemoryDataSize = MemoryDataSize;
	aaa.Mode = Mode;
	aaa.Priority = Priority;
	aaa.M2M = M2M;
	this->Init(aaa);
	this->On();
}

DMA::DMA(const DMA& Other)
{
	this->DMAy_Channelx = Other.DMAy_Channelx;
	this->Increase();
}

void DMA::On()
{
	this->Cmd(ENABLE);   //启动外设
}

void DMA::Off()
{
	this->Cmd(DISABLE);   //关闭外设
}

DMA_Channel_TypeDef* DMA::GetPeriphPointer(void)
{
	return this->DMAy_Channelx;
}

/** @defgroup DMA_Private_Functions
  * @{
  */
DMA::~DMA()
{
	if(this->Decrease()==0)
	{
		this->Off();
		RCC::ClockCmd(*this,DISABLE);
	}
}
/**
  * @brief  Deinitializes the DMAy Channelx registers to their default reset
  *         values.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @retval None
  */
void DMA::DeInit(void)
{
	/* Check the parameters */
	assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));

	/* Disable the selected DMAy Channelx */
	DMAy_Channelx->CCR &= (uint16_t)(~DMA_CCR1_EN);

	/* Reset DMAy Channelx control register */
	DMAy_Channelx->CCR  = 0;

	/* Reset DMAy Channelx remaining bytes register */
	DMAy_Channelx->CNDTR = 0;

	/* Reset DMAy Channelx peripheral address register */
	DMAy_Channelx->CPAR  = 0;

	/* Reset DMAy Channelx memory address register */
	DMAy_Channelx->CMAR = 0;

	if(DMAy_Channelx == DMA1_Channel1)
	{
		/* Reset interrupt pending bits for DMA1 Channel1 */
		DMA1->IFCR |= DMA1_Channel1_IT_Mask;
	}
	else if(DMAy_Channelx == DMA1_Channel2)
	{
		/* Reset interrupt pending bits for DMA1 Channel2 */
		DMA1->IFCR |= DMA1_Channel2_IT_Mask;
	}
	else if(DMAy_Channelx == DMA1_Channel3)
	{
		/* Reset interrupt pending bits for DMA1 Channel3 */
		DMA1->IFCR |= DMA1_Channel3_IT_Mask;
	}
	else if(DMAy_Channelx == DMA1_Channel4)
	{
		/* Reset interrupt pending bits for DMA1 Channel4 */
		DMA1->IFCR |= DMA1_Channel4_IT_Mask;
	}
	else if(DMAy_Channelx == DMA1_Channel5)
	{
		/* Reset interrupt pending bits for DMA1 Channel5 */
		DMA1->IFCR |= DMA1_Channel5_IT_Mask;
	}
	else if(DMAy_Channelx == DMA1_Channel6)
	{
		/* Reset interrupt pending bits for DMA1 Channel6 */
		DMA1->IFCR |= DMA1_Channel6_IT_Mask;
	}
	else if(DMAy_Channelx == DMA1_Channel7)
	{
		/* Reset interrupt pending bits for DMA1 Channel7 */
		DMA1->IFCR |= DMA1_Channel7_IT_Mask;
	}
	else if(DMAy_Channelx == DMA2_Channel1)
	{
		/* Reset interrupt pending bits for DMA2 Channel1 */
		DMA2->IFCR |= DMA2_Channel1_IT_Mask;
	}
	else if(DMAy_Channelx == DMA2_Channel2)
	{
		/* Reset interrupt pending bits for DMA2 Channel2 */
		DMA2->IFCR |= DMA2_Channel2_IT_Mask;
	}
	else if(DMAy_Channelx == DMA2_Channel3)
	{
		/* Reset interrupt pending bits for DMA2 Channel3 */
		DMA2->IFCR |= DMA2_Channel3_IT_Mask;
	}
	else if(DMAy_Channelx == DMA2_Channel4)
	{
		/* Reset interrupt pending bits for DMA2 Channel4 */
		DMA2->IFCR |= DMA2_Channel4_IT_Mask;
	}
	else
	{
		if(DMAy_Channelx == DMA2_Channel5)
		{
			/* Reset interrupt pending bits for DMA2 Channel5 */
			DMA2->IFCR |= DMA2_Channel5_IT_Mask;
		}
	}
}

/**
  * @brief  Initializes the DMAy Channelx according to the specified
  *         parameters in the DMA_InitStruct.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @param  DMA_InitStruct: pointer to a DMA_InitTypeDef structure that
  *         contains the configuration information for the specified DMA Channel.
  * @retval None
  */
void DMA::Init(InitTypeDef &InitStruct)
{
	uint32_t tmpreg = 0;

	/* Check the parameters */
	assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));
	assert_param(IS_DMA_DIR(InitStruct.DIR));
	assert_param(IS_DMA_BUFFER_SIZE(InitStruct.BufferSize));
	assert_param(IS_DMA_PERIPHERAL_INC_STATE(InitStruct.PeripheralInc));
	assert_param(IS_DMA_MEMORY_INC_STATE(InitStruct.MemoryInc));
	assert_param(IS_DMA_PERIPHERAL_DATA_SIZE(InitStruct.PeripheralDataSize));
	assert_param(IS_DMA_MEMORY_DATA_SIZE(InitStruct.MemoryDataSize));
	assert_param(IS_DMA_MODE(InitStruct.Mode));
	assert_param(IS_DMA_PRIORITY(InitStruct.Priority));
	assert_param(IS_DMA_M2M_STATE(InitStruct.M2M));
	RCC::ClockCmd(*this,ENABLE);
	/*--------------------------- DMAy Channelx CCR Configuration -----------------*/
	/* Get the DMAy_Channelx CCR value */
	tmpreg = DMAy_Channelx->CCR;
	/* Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
	tmpreg &= CCR_CLEAR_Mask;
	/* Configure DMAy Channelx: data transfer, data size, priority level and mode */
	/* Set DIR bit according to DIR value */
	/* Set CIRC bit according to Mode value */
	/* Set PINC bit according to PeripheralInc value */
	/* Set MINC bit according to MemoryInc value */
	/* Set PSIZE bits according to PeripheralDataSize value */
	/* Set MSIZE bits according to MemoryDataSize value */
	/* Set PL bits according to Priority value */
	/* Set the MEM2MEM bit according to M2M value */
	tmpreg |= InitStruct.DIR | InitStruct.Mode |
	          InitStruct.PeripheralInc | InitStruct.MemoryInc |
	          InitStruct.PeripheralDataSize | InitStruct.MemoryDataSize |
	          InitStruct.Priority | InitStruct.M2M;

	/* Write to DMAy Channelx CCR */
	DMAy_Channelx->CCR = tmpreg;

	/*--------------------------- DMAy Channelx CNDTR Configuration ---------------*/
	/* Write to DMAy Channelx CNDTR */
	DMAy_Channelx->CNDTR = InitStruct.BufferSize;

	/*--------------------------- DMAy Channelx CPAR Configuration ----------------*/
	/* Write to DMAy Channelx CPAR */
	DMAy_Channelx->CPAR = InitStruct.PeripheralBaseAddr;

	/*--------------------------- DMAy Channelx CMAR Configuration ----------------*/
	/* Write to DMAy Channelx CMAR */
	DMAy_Channelx->CMAR = InitStruct.MemoryBaseAddr;
}

/**
  * @brief  Fills each DMA_InitStruct member with its default value.
  * @param  DMA_InitStruct : pointer to a DMA_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void DMA::StructInit(InitTypeDef &InitStruct)
{
	/*-------------- Reset DMA init structure parameters values ------------------*/
	/* Initialize the PeripheralBaseAddr member */
	InitStruct.PeripheralBaseAddr = 0;
	/* Initialize the MemoryBaseAddr member */
	InitStruct.MemoryBaseAddr = 0;
	/* Initialize the DIR member */
	InitStruct.DIR = DMA_DIR_PeripheralSRC;
	/* Initialize the BufferSize member */
	InitStruct.BufferSize = 0;
	/* Initialize the PeripheralInc member */
	InitStruct.PeripheralInc = DMA_PeripheralInc_Disable;
	/* Initialize the MemoryInc member */
	InitStruct.MemoryInc = DMA_MemoryInc_Disable;
	/* Initialize the PeripheralDataSize member */
	InitStruct.PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	/* Initialize the MemoryDataSize member */
	InitStruct.MemoryDataSize = DMA_MemoryDataSize_Byte;
	/* Initialize the Mode member */
	InitStruct.Mode = DMA_Mode_Normal;
	/* Initialize the Priority member */
	InitStruct.Priority = DMA_Priority_Low;
	/* Initialize the M2M member */
	InitStruct.M2M = DMA_M2M_Disable;
}

/**
  * @brief  Enables or disables the specified DMAy Channelx.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @param  NewState: new state of the DMAy Channelx.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DMA::Cmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		/* Enable the selected DMAy Channelx */
		DMAy_Channelx->CCR |= DMA_CCR1_EN;
	}
	else
	{
		/* Disable the selected DMAy Channelx */
		DMAy_Channelx->CCR &= (uint16_t)(~DMA_CCR1_EN);
	}
}

/**
  * @brief  Enables or disables the specified DMAy Channelx interrupts.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @param  DMA_IT: specifies the DMA interrupts sources to be enabled
  *   or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg DMA_IT_TC:  Transfer complete interrupt mask
  *     @arg DMA_IT_HT:  Half transfer interrupt mask
  *     @arg DMA_IT_TE:  Transfer error interrupt mask
  * @param  NewState: new state of the specified DMA interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DMA::ITConfig(uint32_t DMA_IT, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));
	assert_param(IS_DMA_CONFIG_IT(DMA_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	if(NewState != DISABLE)
	{
		/* Enable the selected DMA interrupts */
		DMAy_Channelx->CCR |= DMA_IT;
	}
	else
	{
		/* Disable the selected DMA interrupts */
		DMAy_Channelx->CCR &= ~DMA_IT;
	}
}

/**
  * @brief  Sets the number of data units in the current DMAy Channelx transfer.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and
  *         x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @param  DataNumber: The number of data units in the current DMAy Channelx
  *         transfer.
  * @note   This function can only be used when the DMAy_Channelx is disabled.
  * @retval None.
  */
void DMA::SetCurrDataCounter(uint16_t DataNumber)
{
	/* Check the parameters */
	assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));

	/*--------------------------- DMAy Channelx CNDTR Configuration ---------------*/
	/* Write to DMAy Channelx CNDTR */
	DMAy_Channelx->CNDTR = DataNumber;
}

/**
  * @brief  Returns the number of remaining data units in the current
  *         DMAy Channelx transfer.
  * @param  DMAy_Channelx: where y can be 1 or 2 to select the DMA and
  *   x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the DMA Channel.
  * @retval The number of remaining data units in the current DMAy Channelx
  *         transfer.
  */
uint16_t DMA::GetCurrDataCounter(void)
{
	/* Check the parameters */
	assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));
	/* Return the number of remaining data units for DMAy Channelx */
	return ((uint16_t)(DMAy_Channelx->CNDTR));
}

/**
  * @brief  Checks whether the specified DMAy Channelx flag is set or not.
  * @param  DMAy_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg DMA1_FLAG_GL1: DMA1 Channel1 global flag.
  *     @arg DMA1_FLAG_TC1: DMA1 Channel1 transfer complete flag.
  *     @arg DMA1_FLAG_HT1: DMA1 Channel1 half transfer flag.
  *     @arg DMA1_FLAG_TE1: DMA1 Channel1 transfer error flag.
  *     @arg DMA1_FLAG_GL2: DMA1 Channel2 global flag.
  *     @arg DMA1_FLAG_TC2: DMA1 Channel2 transfer complete flag.
  *     @arg DMA1_FLAG_HT2: DMA1 Channel2 half transfer flag.
  *     @arg DMA1_FLAG_TE2: DMA1 Channel2 transfer error flag.
  *     @arg DMA1_FLAG_GL3: DMA1 Channel3 global flag.
  *     @arg DMA1_FLAG_TC3: DMA1 Channel3 transfer complete flag.
  *     @arg DMA1_FLAG_HT3: DMA1 Channel3 half transfer flag.
  *     @arg DMA1_FLAG_TE3: DMA1 Channel3 transfer error flag.
  *     @arg DMA1_FLAG_GL4: DMA1 Channel4 global flag.
  *     @arg DMA1_FLAG_TC4: DMA1 Channel4 transfer complete flag.
  *     @arg DMA1_FLAG_HT4: DMA1 Channel4 half transfer flag.
  *     @arg DMA1_FLAG_TE4: DMA1 Channel4 transfer error flag.
  *     @arg DMA1_FLAG_GL5: DMA1 Channel5 global flag.
  *     @arg DMA1_FLAG_TC5: DMA1 Channel5 transfer complete flag.
  *     @arg DMA1_FLAG_HT5: DMA1 Channel5 half transfer flag.
  *     @arg DMA1_FLAG_TE5: DMA1 Channel5 transfer error flag.
  *     @arg DMA1_FLAG_GL6: DMA1 Channel6 global flag.
  *     @arg DMA1_FLAG_TC6: DMA1 Channel6 transfer complete flag.
  *     @arg DMA1_FLAG_HT6: DMA1 Channel6 half transfer flag.
  *     @arg DMA1_FLAG_TE6: DMA1 Channel6 transfer error flag.
  *     @arg DMA1_FLAG_GL7: DMA1 Channel7 global flag.
  *     @arg DMA1_FLAG_TC7: DMA1 Channel7 transfer complete flag.
  *     @arg DMA1_FLAG_HT7: DMA1 Channel7 half transfer flag.
  *     @arg DMA1_FLAG_TE7: DMA1 Channel7 transfer error flag.
  *     @arg DMA2_FLAG_GL1: DMA2 Channel1 global flag.
  *     @arg DMA2_FLAG_TC1: DMA2 Channel1 transfer complete flag.
  *     @arg DMA2_FLAG_HT1: DMA2 Channel1 half transfer flag.
  *     @arg DMA2_FLAG_TE1: DMA2 Channel1 transfer error flag.
  *     @arg DMA2_FLAG_GL2: DMA2 Channel2 global flag.
  *     @arg DMA2_FLAG_TC2: DMA2 Channel2 transfer complete flag.
  *     @arg DMA2_FLAG_HT2: DMA2 Channel2 half transfer flag.
  *     @arg DMA2_FLAG_TE2: DMA2 Channel2 transfer error flag.
  *     @arg DMA2_FLAG_GL3: DMA2 Channel3 global flag.
  *     @arg DMA2_FLAG_TC3: DMA2 Channel3 transfer complete flag.
  *     @arg DMA2_FLAG_HT3: DMA2 Channel3 half transfer flag.
  *     @arg DMA2_FLAG_TE3: DMA2 Channel3 transfer error flag.
  *     @arg DMA2_FLAG_GL4: DMA2 Channel4 global flag.
  *     @arg DMA2_FLAG_TC4: DMA2 Channel4 transfer complete flag.
  *     @arg DMA2_FLAG_HT4: DMA2 Channel4 half transfer flag.
  *     @arg DMA2_FLAG_TE4: DMA2 Channel4 transfer error flag.
  *     @arg DMA2_FLAG_GL5: DMA2 Channel5 global flag.
  *     @arg DMA2_FLAG_TC5: DMA2 Channel5 transfer complete flag.
  *     @arg DMA2_FLAG_HT5: DMA2 Channel5 half transfer flag.
  *     @arg DMA2_FLAG_TE5: DMA2 Channel5 transfer error flag.
  * @retval The new state of DMAy_FLAG (SET or RESET).
  */
FlagStatus DMA::GetFlagStatus(uint32_t DMAy_FLAG)
{
	FlagStatus bitstatus = RESET;
	uint32_t tmpreg = 0;

	/* Check the parameters */
	assert_param(IS_DMA_GET_FLAG(DMAy_FLAG));

	/* Calculate the used DMAy */
	if((DMAy_FLAG & FLAG_Mask) != (uint32_t)RESET)
	{
		/* Get DMA2 ISR register value */
		tmpreg = DMA2->ISR ;
	}
	else
	{
		/* Get DMA1 ISR register value */
		tmpreg = DMA1->ISR ;
	}

	/* Check the status of the specified DMAy flag */
	if((tmpreg & DMAy_FLAG) != (uint32_t)RESET)
	{
		/* DMAy_FLAG is set */
		bitstatus = SET;
	}
	else
	{
		/* DMAy_FLAG is reset */
		bitstatus = RESET;
	}

	/* Return the DMAy_FLAG status */
	return  bitstatus;
}

/**
  * @brief  Clears the DMAy Channelx's pending flags.
  * @param  DMAy_FLAG: specifies the flag to clear.
  *   This parameter can be any combination (for the same DMA) of the following values:
  *     @arg DMA1_FLAG_GL1: DMA1 Channel1 global flag.
  *     @arg DMA1_FLAG_TC1: DMA1 Channel1 transfer complete flag.
  *     @arg DMA1_FLAG_HT1: DMA1 Channel1 half transfer flag.
  *     @arg DMA1_FLAG_TE1: DMA1 Channel1 transfer error flag.
  *     @arg DMA1_FLAG_GL2: DMA1 Channel2 global flag.
  *     @arg DMA1_FLAG_TC2: DMA1 Channel2 transfer complete flag.
  *     @arg DMA1_FLAG_HT2: DMA1 Channel2 half transfer flag.
  *     @arg DMA1_FLAG_TE2: DMA1 Channel2 transfer error flag.
  *     @arg DMA1_FLAG_GL3: DMA1 Channel3 global flag.
  *     @arg DMA1_FLAG_TC3: DMA1 Channel3 transfer complete flag.
  *     @arg DMA1_FLAG_HT3: DMA1 Channel3 half transfer flag.
  *     @arg DMA1_FLAG_TE3: DMA1 Channel3 transfer error flag.
  *     @arg DMA1_FLAG_GL4: DMA1 Channel4 global flag.
  *     @arg DMA1_FLAG_TC4: DMA1 Channel4 transfer complete flag.
  *     @arg DMA1_FLAG_HT4: DMA1 Channel4 half transfer flag.
  *     @arg DMA1_FLAG_TE4: DMA1 Channel4 transfer error flag.
  *     @arg DMA1_FLAG_GL5: DMA1 Channel5 global flag.
  *     @arg DMA1_FLAG_TC5: DMA1 Channel5 transfer complete flag.
  *     @arg DMA1_FLAG_HT5: DMA1 Channel5 half transfer flag.
  *     @arg DMA1_FLAG_TE5: DMA1 Channel5 transfer error flag.
  *     @arg DMA1_FLAG_GL6: DMA1 Channel6 global flag.
  *     @arg DMA1_FLAG_TC6: DMA1 Channel6 transfer complete flag.
  *     @arg DMA1_FLAG_HT6: DMA1 Channel6 half transfer flag.
  *     @arg DMA1_FLAG_TE6: DMA1 Channel6 transfer error flag.
  *     @arg DMA1_FLAG_GL7: DMA1 Channel7 global flag.
  *     @arg DMA1_FLAG_TC7: DMA1 Channel7 transfer complete flag.
  *     @arg DMA1_FLAG_HT7: DMA1 Channel7 half transfer flag.
  *     @arg DMA1_FLAG_TE7: DMA1 Channel7 transfer error flag.
  *     @arg DMA2_FLAG_GL1: DMA2 Channel1 global flag.
  *     @arg DMA2_FLAG_TC1: DMA2 Channel1 transfer complete flag.
  *     @arg DMA2_FLAG_HT1: DMA2 Channel1 half transfer flag.
  *     @arg DMA2_FLAG_TE1: DMA2 Channel1 transfer error flag.
  *     @arg DMA2_FLAG_GL2: DMA2 Channel2 global flag.
  *     @arg DMA2_FLAG_TC2: DMA2 Channel2 transfer complete flag.
  *     @arg DMA2_FLAG_HT2: DMA2 Channel2 half transfer flag.
  *     @arg DMA2_FLAG_TE2: DMA2 Channel2 transfer error flag.
  *     @arg DMA2_FLAG_GL3: DMA2 Channel3 global flag.
  *     @arg DMA2_FLAG_TC3: DMA2 Channel3 transfer complete flag.
  *     @arg DMA2_FLAG_HT3: DMA2 Channel3 half transfer flag.
  *     @arg DMA2_FLAG_TE3: DMA2 Channel3 transfer error flag.
  *     @arg DMA2_FLAG_GL4: DMA2 Channel4 global flag.
  *     @arg DMA2_FLAG_TC4: DMA2 Channel4 transfer complete flag.
  *     @arg DMA2_FLAG_HT4: DMA2 Channel4 half transfer flag.
  *     @arg DMA2_FLAG_TE4: DMA2 Channel4 transfer error flag.
  *     @arg DMA2_FLAG_GL5: DMA2 Channel5 global flag.
  *     @arg DMA2_FLAG_TC5: DMA2 Channel5 transfer complete flag.
  *     @arg DMA2_FLAG_HT5: DMA2 Channel5 half transfer flag.
  *     @arg DMA2_FLAG_TE5: DMA2 Channel5 transfer error flag.
  * @retval None
  */
void DMA::ClearFlag(uint32_t DMAy_FLAG)
{
	/* Check the parameters */
	assert_param(IS_DMA_CLEAR_FLAG(DMAy_FLAG));

	/* Calculate the used DMAy */
	if((DMAy_FLAG & FLAG_Mask) != (uint32_t)RESET)
	{
		/* Clear the selected DMAy flags */
		DMA2->IFCR = DMAy_FLAG;
	}
	else
	{
		/* Clear the selected DMAy flags */
		DMA1->IFCR = DMAy_FLAG;
	}
}

/**
  * @brief  Checks whether the specified DMAy Channelx interrupt has occurred or not.
  * @param  DMAy_IT: specifies the DMAy interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg DMA1_IT_GL1: DMA1 Channel1 global interrupt.
  *     @arg DMA1_IT_TC1: DMA1 Channel1 transfer complete interrupt.
  *     @arg DMA1_IT_HT1: DMA1 Channel1 half transfer interrupt.
  *     @arg DMA1_IT_TE1: DMA1 Channel1 transfer error interrupt.
  *     @arg DMA1_IT_GL2: DMA1 Channel2 global interrupt.
  *     @arg DMA1_IT_TC2: DMA1 Channel2 transfer complete interrupt.
  *     @arg DMA1_IT_HT2: DMA1 Channel2 half transfer interrupt.
  *     @arg DMA1_IT_TE2: DMA1 Channel2 transfer error interrupt.
  *     @arg DMA1_IT_GL3: DMA1 Channel3 global interrupt.
  *     @arg DMA1_IT_TC3: DMA1 Channel3 transfer complete interrupt.
  *     @arg DMA1_IT_HT3: DMA1 Channel3 half transfer interrupt.
  *     @arg DMA1_IT_TE3: DMA1 Channel3 transfer error interrupt.
  *     @arg DMA1_IT_GL4: DMA1 Channel4 global interrupt.
  *     @arg DMA1_IT_TC4: DMA1 Channel4 transfer complete interrupt.
  *     @arg DMA1_IT_HT4: DMA1 Channel4 half transfer interrupt.
  *     @arg DMA1_IT_TE4: DMA1 Channel4 transfer error interrupt.
  *     @arg DMA1_IT_GL5: DMA1 Channel5 global interrupt.
  *     @arg DMA1_IT_TC5: DMA1 Channel5 transfer complete interrupt.
  *     @arg DMA1_IT_HT5: DMA1 Channel5 half transfer interrupt.
  *     @arg DMA1_IT_TE5: DMA1 Channel5 transfer error interrupt.
  *     @arg DMA1_IT_GL6: DMA1 Channel6 global interrupt.
  *     @arg DMA1_IT_TC6: DMA1 Channel6 transfer complete interrupt.
  *     @arg DMA1_IT_HT6: DMA1 Channel6 half transfer interrupt.
  *     @arg DMA1_IT_TE6: DMA1 Channel6 transfer error interrupt.
  *     @arg DMA1_IT_GL7: DMA1 Channel7 global interrupt.
  *     @arg DMA1_IT_TC7: DMA1 Channel7 transfer complete interrupt.
  *     @arg DMA1_IT_HT7: DMA1 Channel7 half transfer interrupt.
  *     @arg DMA1_IT_TE7: DMA1 Channel7 transfer error interrupt.
  *     @arg DMA2_IT_GL1: DMA2 Channel1 global interrupt.
  *     @arg DMA2_IT_TC1: DMA2 Channel1 transfer complete interrupt.
  *     @arg DMA2_IT_HT1: DMA2 Channel1 half transfer interrupt.
  *     @arg DMA2_IT_TE1: DMA2 Channel1 transfer error interrupt.
  *     @arg DMA2_IT_GL2: DMA2 Channel2 global interrupt.
  *     @arg DMA2_IT_TC2: DMA2 Channel2 transfer complete interrupt.
  *     @arg DMA2_IT_HT2: DMA2 Channel2 half transfer interrupt.
  *     @arg DMA2_IT_TE2: DMA2 Channel2 transfer error interrupt.
  *     @arg DMA2_IT_GL3: DMA2 Channel3 global interrupt.
  *     @arg DMA2_IT_TC3: DMA2 Channel3 transfer complete interrupt.
  *     @arg DMA2_IT_HT3: DMA2 Channel3 half transfer interrupt.
  *     @arg DMA2_IT_TE3: DMA2 Channel3 transfer error interrupt.
  *     @arg DMA2_IT_GL4: DMA2 Channel4 global interrupt.
  *     @arg DMA2_IT_TC4: DMA2 Channel4 transfer complete interrupt.
  *     @arg DMA2_IT_HT4: DMA2 Channel4 half transfer interrupt.
  *     @arg DMA2_IT_TE4: DMA2 Channel4 transfer error interrupt.
  *     @arg DMA2_IT_GL5: DMA2 Channel5 global interrupt.
  *     @arg DMA2_IT_TC5: DMA2 Channel5 transfer complete interrupt.
  *     @arg DMA2_IT_HT5: DMA2 Channel5 half transfer interrupt.
  *     @arg DMA2_IT_TE5: DMA2 Channel5 transfer error interrupt.
  * @retval The new state of DMAy_IT (SET or RESET).
  */
ITStatus DMA::GetITStatus(uint32_t DMAy_IT)
{
	ITStatus bitstatus = RESET;
	uint32_t tmpreg = 0;

	/* Check the parameters */
	assert_param(IS_DMA_GET_IT(DMAy_IT));

	/* Calculate the used DMA */
	if((DMAy_IT & FLAG_Mask) != (uint32_t)RESET)
	{
		/* Get DMA2 ISR register value */
		tmpreg = DMA2->ISR;
	}
	else
	{
		/* Get DMA1 ISR register value */
		tmpreg = DMA1->ISR;
	}

	/* Check the status of the specified DMAy interrupt */
	if((tmpreg & DMAy_IT) != (uint32_t)RESET)
	{
		/* DMAy_IT is set */
		bitstatus = SET;
	}
	else
	{
		/* DMAy_IT is reset */
		bitstatus = RESET;
	}
	/* Return the DMA_IT status */
	return  bitstatus;
}

/**
  * @brief  Clears the DMAy Channelx's interrupt pending bits.
  * @param  DMAy_IT: specifies the DMAy interrupt pending bit to clear.
  *   This parameter can be any combination (for the same DMA) of the following values:
  *     @arg DMA1_IT_GL1: DMA1 Channel1 global interrupt.
  *     @arg DMA1_IT_TC1: DMA1 Channel1 transfer complete interrupt.
  *     @arg DMA1_IT_HT1: DMA1 Channel1 half transfer interrupt.
  *     @arg DMA1_IT_TE1: DMA1 Channel1 transfer error interrupt.
  *     @arg DMA1_IT_GL2: DMA1 Channel2 global interrupt.
  *     @arg DMA1_IT_TC2: DMA1 Channel2 transfer complete interrupt.
  *     @arg DMA1_IT_HT2: DMA1 Channel2 half transfer interrupt.
  *     @arg DMA1_IT_TE2: DMA1 Channel2 transfer error interrupt.
  *     @arg DMA1_IT_GL3: DMA1 Channel3 global interrupt.
  *     @arg DMA1_IT_TC3: DMA1 Channel3 transfer complete interrupt.
  *     @arg DMA1_IT_HT3: DMA1 Channel3 half transfer interrupt.
  *     @arg DMA1_IT_TE3: DMA1 Channel3 transfer error interrupt.
  *     @arg DMA1_IT_GL4: DMA1 Channel4 global interrupt.
  *     @arg DMA1_IT_TC4: DMA1 Channel4 transfer complete interrupt.
  *     @arg DMA1_IT_HT4: DMA1 Channel4 half transfer interrupt.
  *     @arg DMA1_IT_TE4: DMA1 Channel4 transfer error interrupt.
  *     @arg DMA1_IT_GL5: DMA1 Channel5 global interrupt.
  *     @arg DMA1_IT_TC5: DMA1 Channel5 transfer complete interrupt.
  *     @arg DMA1_IT_HT5: DMA1 Channel5 half transfer interrupt.
  *     @arg DMA1_IT_TE5: DMA1 Channel5 transfer error interrupt.
  *     @arg DMA1_IT_GL6: DMA1 Channel6 global interrupt.
  *     @arg DMA1_IT_TC6: DMA1 Channel6 transfer complete interrupt.
  *     @arg DMA1_IT_HT6: DMA1 Channel6 half transfer interrupt.
  *     @arg DMA1_IT_TE6: DMA1 Channel6 transfer error interrupt.
  *     @arg DMA1_IT_GL7: DMA1 Channel7 global interrupt.
  *     @arg DMA1_IT_TC7: DMA1 Channel7 transfer complete interrupt.
  *     @arg DMA1_IT_HT7: DMA1 Channel7 half transfer interrupt.
  *     @arg DMA1_IT_TE7: DMA1 Channel7 transfer error interrupt.
  *     @arg DMA2_IT_GL1: DMA2 Channel1 global interrupt.
  *     @arg DMA2_IT_TC1: DMA2 Channel1 transfer complete interrupt.
  *     @arg DMA2_IT_HT1: DMA2 Channel1 half transfer interrupt.
  *     @arg DMA2_IT_TE1: DMA2 Channel1 transfer error interrupt.
  *     @arg DMA2_IT_GL2: DMA2 Channel2 global interrupt.
  *     @arg DMA2_IT_TC2: DMA2 Channel2 transfer complete interrupt.
  *     @arg DMA2_IT_HT2: DMA2 Channel2 half transfer interrupt.
  *     @arg DMA2_IT_TE2: DMA2 Channel2 transfer error interrupt.
  *     @arg DMA2_IT_GL3: DMA2 Channel3 global interrupt.
  *     @arg DMA2_IT_TC3: DMA2 Channel3 transfer complete interrupt.
  *     @arg DMA2_IT_HT3: DMA2 Channel3 half transfer interrupt.
  *     @arg DMA2_IT_TE3: DMA2 Channel3 transfer error interrupt.
  *     @arg DMA2_IT_GL4: DMA2 Channel4 global interrupt.
  *     @arg DMA2_IT_TC4: DMA2 Channel4 transfer complete interrupt.
  *     @arg DMA2_IT_HT4: DMA2 Channel4 half transfer interrupt.
  *     @arg DMA2_IT_TE4: DMA2 Channel4 transfer error interrupt.
  *     @arg DMA2_IT_GL5: DMA2 Channel5 global interrupt.
  *     @arg DMA2_IT_TC5: DMA2 Channel5 transfer complete interrupt.
  *     @arg DMA2_IT_HT5: DMA2 Channel5 half transfer interrupt.
  *     @arg DMA2_IT_TE5: DMA2 Channel5 transfer error interrupt.
  * @retval None
  */
void DMA::ClearITPendingBit(uint32_t DMAy_IT)
{
	/* Check the parameters */
	assert_param(IS_DMA_CLEAR_IT(DMAy_IT));

	/* Calculate the used DMAy */
	if((DMAy_IT & FLAG_Mask) != (uint32_t)RESET)
	{
		/* Clear the selected DMAy interrupt pending bits */
		DMA2->IFCR = DMAy_IT;
	}
	else
	{
		/* Clear the selected DMAy interrupt pending bits */
		DMA1->IFCR = DMAy_IT;
	}
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
