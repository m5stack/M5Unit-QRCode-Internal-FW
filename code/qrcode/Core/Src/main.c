/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_ex.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS 0x21
#define FIRMWARE_VERSION 3
#define APPLICATION_ADDRESS     ((uint32_t)0x08001000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buffer[BUF_SIZE] = {0};
uint8_t uart_tx_buffer[TX_BUF_SIZE] = {0};
uint8_t i2c_address[1] = {0};
uint8_t uart_dma_init_flag = 0;
uint32_t decode_length = 0;
uint8_t trigger_flag = 0;
uint8_t data_ready_step = 0;
uint8_t data_ready = 0;
volatile uint8_t fm_version = FIRMWARE_VERSION;
volatile uint8_t flag_jump_bootloader = 0;
volatile uint32_t jump_bootloader_timeout = 0;
volatile uint8_t trigger_mode = 0;
volatile uint32_t i2c_stop_timeout_delay = 0;

volatile uint8_t uart_mode_flag = 0;

volatile uint8_t is_i2c_enable = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//????????SRAM???
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //??? SRAM ??? 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

void i2c_port_set_to_input(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /*Configure GPIO pin Output Level */
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9 | LL_GPIO_PIN_10);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);  
}

void user_i2c_init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA9   ------> I2C1_SCL
  PA10   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x0000020B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = i2c_address[0]<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */
  set_i2c_slave_address(i2c_address[0]);
  /* USER CODE END I2C1_Init 2 */

}

void i2c_address_write_to_flash(void) 
{   
  writeMessageToFlash(i2c_address , 1);   
}

void i2c_address_read_from_flash(void) 
{   
  if (!(readPackedMessageFromFlash(i2c_address, 1))) {
    i2c_address[0] = I2C_ADDRESS;
    i2c_address_write_to_flash();
  }
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len)
{
  uint16_t reg_address = 0;

  reg_address = (rx_data[0] | (rx_data[1] << 8));
  if (len > 2) {
    if (!is_i2c_enable) {
      MX_DMA_Init();
      MX_USART1_UART_Init();
      __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
      HAL_UART_Receive_DMA(&huart1, buffer, BUF_SIZE);     
      uart_tx_buffer[0] = 0x21;
      uart_tx_buffer[1] = 0x61;
      uart_tx_buffer[2] = 0x41;
      uart_tx_buffer[3] = 0x05;
      data_ready_step = 0;
      HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 4);    
      is_i2c_enable = 1;
    }    
    if (reg_address == 0)
    {
      if (rx_data[2]) {
        uart_tx_buffer[0] = 0x32;
        uart_tx_buffer[1] = 0x75;
        uart_tx_buffer[2] = 0x01;
        data_ready_step = 1;
        HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 3);
      } else {
        uart_tx_buffer[0] = 0x32;
        uart_tx_buffer[1] = 0x75;
        uart_tx_buffer[2] = 0x02;
				data_ready_step = 0;
        HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 3);        
      }
    } 
    else if (reg_address == 0x00FD)
    {
      if (rx_data[2] == 1) {
        flag_jump_bootloader = 1;
        if (flag_jump_bootloader) {
          LL_I2C_DeInit(I2C1);
          LL_I2C_DisableAutoEndMode(I2C1);
          LL_I2C_Disable(I2C1);
          LL_I2C_DisableIT_ADDR(I2C1);
          HAL_UART_MspDeInit(&huart1);
          __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
          i2c_port_set_to_input();
          while ((!!(GPIOA->IDR & LL_GPIO_PIN_9)) || (!!(GPIOA->IDR & LL_GPIO_PIN_10)))
          {
            jump_bootloader_timeout++;
            if (jump_bootloader_timeout >= 60000) {
              flag_jump_bootloader = 0;
              break;
            }
          }
          if (jump_bootloader_timeout < 60000) {
            NVIC_SystemReset();
          } else {
            // NVIC_SystemReset();
            user_i2c_init();
            i2c1_it_enable();
            MX_DMA_Init(); 
            MX_USART1_UART_Init(); 
            __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
            HAL_UART_Receive_DMA(&huart1, buffer, BUF_SIZE);                    
            jump_bootloader_timeout = 0;
          }
        }        
      }
    } 
    else if (reg_address == 0x00FF)
    {
      if (rx_data[2] < 128) {
        i2c_address[0] = rx_data[2];
        i2c_address_write_to_flash();
        user_i2c_init();
      }
    } 
    else if (reg_address == 0x0010)
    {
      if (rx_data[2] == 0)
        data_ready = rx_data[2];
    } 
    else if (reg_address == 0x0030)
    {
      if (rx_data[2] == 0) {
        trigger_mode = 0;
        uart_tx_buffer[0] = 0x21;
        uart_tx_buffer[1] = 0x61;
        uart_tx_buffer[2] = 0x41;
        uart_tx_buffer[3] = 0x05;
        HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 4);         
      }
      else {
        trigger_mode = 1;
        uart_tx_buffer[0] = 0x21;
        uart_tx_buffer[1] = 0x61;
        uart_tx_buffer[2] = 0x41;
        uart_tx_buffer[3] = 0x00;
        HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 4);          
      }
    } 
  }
  else if (len == 2) {
    if (!is_i2c_enable) {
      MX_DMA_Init();
      MX_USART1_UART_Init();
      __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
      HAL_UART_Receive_DMA(&huart1, buffer, BUF_SIZE);     
      uart_tx_buffer[0] = 0x21;
      uart_tx_buffer[1] = 0x61;
      uart_tx_buffer[2] = 0x41;
      uart_tx_buffer[3] = 0x05;
      data_ready_step = 0;
      HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 4);    
      is_i2c_enable = 1;
    }    
    if (reg_address == 0x0010)
    {
      i2c1_set_send_data((uint8_t*)&data_ready, 1);
    } 
    else if (reg_address == 0x0020)
    {
      i2c1_set_send_data((uint8_t*)&decode_length, 2);
    } 
    else if (reg_address == 0x0040)
    {
      uint8_t button_status = (!!(GPIOB->IDR & LL_GPIO_PIN_1));
      i2c1_set_send_data((uint8_t*)&button_status, 1);
    } 
    else if (reg_address == 0x00FE) 
    {
      i2c1_set_send_data((uint8_t *)&fm_version, 1);
    }     
    else if (reg_address == 0x00FF) 
    {
      i2c1_set_send_data(i2c_address, 1); 
    }     
    else if ((reg_address >= 0x1000) && (reg_address <= 0x13FF))
    {
      i2c1_set_send_data(buffer, decode_length < BUF_SIZE ? decode_length : BUF_SIZE);
      data_ready = 0;
    }
    else if (reg_address == 0x0030)
    {
      i2c1_set_send_data((uint8_t *)&trigger_mode, 1);
    }      
  }
 
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // MX_DMA_Init();
  // MX_I2C1_Init();
  // MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // if(!(!!(GPIOB->IDR & LL_GPIO_PIN_1))) {
  //   HAL_Delay(2000);
  //   if(!(!!(GPIOB->IDR & LL_GPIO_PIN_1))) {
  //     uart_mode_flag = 1;
  //   }
  // }
  // if (!uart_mode_flag) {
  //   MX_DMA_Init();
  //   MX_USART1_UART_Init();
  //   __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  //   HAL_UART_Receive_DMA(&huart1, buffer, BUF_SIZE);      
  // }
  i2c_address_read_from_flash();
  user_i2c_init();    
  i2c1_it_enable();
  // if (!uart_mode_flag) {
  //   uart_tx_buffer[0] = 0x21;
  //   uart_tx_buffer[1] = 0x61;
  //   uart_tx_buffer[2] = 0x41;
  //   uart_tx_buffer[3] = 0x05;
  //   data_ready_step = 0;
  //   HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 4);     
  // }  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    i2c_timeout_counter = 0;
    if (i2c_stop_timeout_flag) {
      if (i2c_stop_timeout_delay < HAL_GetTick()) {
        i2c_stop_timeout_counter++;
        i2c_stop_timeout_delay = HAL_GetTick() + 10;
      }
    }
    if (i2c_stop_timeout_counter > 50) {
      LL_I2C_DeInit(I2C1);
      LL_I2C_DisableAutoEndMode(I2C1);
      LL_I2C_Disable(I2C1);
      LL_I2C_DisableIT_ADDR(I2C1);     
      user_i2c_init(); 
      i2c1_it_enable();
      HAL_Delay(500);
    }     
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_HSI);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
