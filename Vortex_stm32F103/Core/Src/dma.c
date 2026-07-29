/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of the DMA instances.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "dma.h"
#include "usart.h"

DMA_HandleTypeDef hdma_usart1_rx;

/* DMA init function */
void MX_DMA_Init(void)
{
  hdma_usart1_rx.Instance = DMA1_Channel5;
  hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_rx.Init.Mode = DMA_NORMAL;
  hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
}

void HAL_DMA_MspInit(DMA_HandleTypeDef* dmaHandle)
{
  if(dmaHandle->Instance==DMA1_Channel5)
  {
    /* DMA1 clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA1_Channel5 interrupt Init */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  }
}

void HAL_DMA_MspDeInit(DMA_HandleTypeDef* dmaHandle)
{
  if(dmaHandle->Instance==DMA1_Channel5)
  {
    /* Peripheral clock disable */
    __HAL_RCC_DMA1_CLK_DISABLE();

    /* DMA1_Channel5 interrupt DeInit */
    HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
  }
}
