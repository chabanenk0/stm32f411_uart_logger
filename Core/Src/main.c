/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "crsf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CRSF_Packet crsf_packet;
struct crsfPayloadAttitude_s CRSF_Attitude;
struct crsfPayloadLinkstatistics_s CRSF_LinkStatistics;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void clear_link_statistics(struct crsfPayloadLinkstatistics_s* crsf_link_statistics);
void processCrsfFrame(uint8_t * data, uint8_t device_id, uint8_t frame_size, uint8_t crc, uint32_t crc_failures_count, struct crsfPacket_s *crsf_packet, struct crsfPayloadLinkstatistics_s* crsf_link_statistics, struct crsfPayloadAttitude_s * crsf_attitude);
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define BUFFER_SIZE 600
#define MEMORY_LIMIT  412000

uint32_t t_received = 0;
uint32_t t_transmitted = 0;
uint16_t buffer_position, previously_read;
uint8_t device_id;
uint8_t frame_size;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    uint8_t data[BUFFER_SIZE];
    uint8_t crc;
    uint64_t i = 0;

    for(i = 0; i < BUFFER_SIZE; i++) {
        data[i] = 0;
    }

    i = 0;
    uint32_t crc_failures_count = 0;
    clear_link_statistics(&CRSF_LinkStatistics);
    clear_attitude(&CRSF_Attitude);
    buffer_position = 0;
    previously_read = 0;
    previously_read = readFromSource(1, data, buffer_position, previously_read, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        previously_read = readFromSource(1, data, buffer_position + 1, previously_read, 1);
        device_id = data[buffer_position];
        frame_size = data[buffer_position + 1];
        printf("frame size: %d\n", frame_size);
        previously_read = readFromSource(1, data, buffer_position + 2, previously_read, frame_size);
        crc = data[buffer_position + frame_size + 1];
        uint8_t crc_fact = calculateCRC(data, buffer_position + 2, frame_size);

        if (crc_fact == crc) {
            processCrsfFrame(data + buffer_position + 2, device_id, frame_size, crc, crc_failures_count, &crsf_packet, &CRSF_LinkStatistics, &CRSF_Attitude);
            //buffer_position = buffer_position + frame_size + 2;
            buffer_position = 0;
            previously_read = 0;
            printf("crc ok, crc = %d, frame_size: %d, dump_position = %d (%X), buffer_position: %d\n", crc, frame_size, 0, 0, buffer_position);
        } else {
            crc_failures_count++;
            // search for another beginning
            printf("CRC failed: crc fact: %d,  crc read: %d\n", crc_fact, crc);
            printf("frame_size: %d, dump_position = %d (%X), buffer_position: %d\n", frame_size, 0, 0, buffer_position);
            uint8_t found = 0;

            for(uint16_t i = buffer_position + 1; i < buffer_position + frame_size; i++) {
                printf("Searching for new beginning in buffer, i=%d, data[i]=%0X, dump position: %d", i, data[i], 0);

                if (data[i] == CRSF_SYNC_BYTE) {
                    buffer_position = i;
                    printf("- Found!");
                    found = 1;
                    break;
                }

                printf("\n");
            }

            if (!found) {
                buffer_position = 0;

                do {
                    readFromSource(1, data, buffer_position, 0, 1);
                    printf("Searching for new beginning in file, dump position: %X (%d), char: %x", 0, 0, data[buffer_position]);
                } while(data[buffer_position] != CRSF_SYNC_BYTE);

                previously_read = 1;
            }
        }

      if (t_received > MEMORY_LIMIT) {
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(1000);
        }
      }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 420000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 420000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint32_t address_received = 0x08003a00;
uint32_t address_transmitted = 0x08103a00;


PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
void store_received_data(char * cData) 
{
    uint32_t tick = //HAL_GetTick();//__HAL_TIM_GET_COUNTER(&htim1);
    write_to_flash(t_received, address_received, tick, cData);
    t_received = t_received + BUFFER_SIZE;
}

void store_transmitted_data(char cData) {
    uint32_t tick = HAL_GetTick();//__HAL_TIM_GET_COUNTER(&htim1);
    write_to_flash(t_transmitted, address_transmitted, tick, cData);
    t_transmitted++;
}

void handle_recieved_data(char cData) {

}

int write_to_flash(uint32_t t, uint32_t address_beginning, uint32_t tick, char * data)
{
      /* Unlock the Flash to enable the flash control register access *************/
       HAL_FLASH_Unlock();

       /* Erase the user Flash area*/

      uint32_t StartPageAddress;
      uint64_t i;

      for(i = 0; i < BUFFER_SIZE; i++) {
          StartPageAddress = address_beginning + t + i;

          if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, StartPageAddress, (uint64_t) data[i]) != HAL_OK) {
              return HAL_FLASH_GetError ();
          }
      }

      /* Lock the Flash to disable the flash control register access (recommended
          to protect the FLASH memory against possible unwanted operation) *********/
      HAL_FLASH_Lock();

      return 0;
}

int data_read(uint8_t * data, int n, int sourceId)
{
    UART_HandleTypeDef huart;
    if (1 == sourceId) {
        huart = huart2;
    } else if (2 == sourceId) {
        huart = huart1;
    } else {
        return -1;
    }

    if (HAL_UART_Receive(&huart, (uint8_t*) data, BUFFER_SIZE, 20000) != HAL_OK) {
        return 1;
    }

    return -2;
}

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
