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
#include <stdio.h>
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void clear_link_statistics(struct crsfPayloadLinkstatistics_s* crsf_link_statistics);
void processCrsfFrame(uint8_t * data, uint8_t device_id, uint8_t frame_size, uint8_t crc, uint32_t crc_failures_count, struct crsfPacket_s *crsf_packet, struct crsfPayloadLinkstatistics_s* crsf_link_statistics, struct crsfPayloadAttitude_s * crsf_attitude);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char buffer[256];
//char verbose = 0;
#define MEMORY_LIMIT  412000

uint32_t t_received = 0;
uint32_t t_transmitted = 0;
uint16_t buffer_position, previously_read;
uint8_t device_id;
uint8_t frame_size;

typedef struct data_for_flash {
  uint8_t beginning_marker;
  uint32_t tick;
  uint16_t rx_pitch;
  uint16_t rx_roll;
  uint16_t rx_yaw;
  uint16_t rx_trottle;
  uint16_t rx_arm;
  uint pitch;
  uint roll;
  uint yaw;
  uint8_t uplink_quality;
  uint8_t downlink_quality;
  char flight_mode[4];
} data_for_flash_s;

struct data_for_flash data_for_flash_obj;

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(5000);
  if (verbose > 1) {
      printf("Delay passed...\nHello, world!");
  }

    uint8_t data[BUFFER_SIZE];
    uint8_t crc;
    uint64_t i = 0, ii = 0;

    for(i = 0; i < BUFFER_SIZE; i++) {
        data[i] = 0;
    }

    i = 0;
    uint32_t crc_failures_count = 0;
    clear_link_statistics(&CRSF_LinkStatistics);
    clear_attitude(&CRSF_Attitude);
    buffer_position = 0;
    previously_read = 0;
    uint32_t position = 0;

    if (verbose > 1) {
        printf("\nStarting receiving data from main...\n");
    }

    do {
        readFromSource(1, data, buffer_position, 0, 1);
        if (verbose > 1) {
            printf("Searching for new beginning char: %x", (int) data[buffer_position]);
        }
    } while(data[buffer_position] != CRSF_SYNC_BYTE);

    previously_read++;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if (verbose > 1) {
            printf("Main: infinite loop...\n");
        }

        previously_read = readFromSource(1, data, buffer_position + 1, previously_read, 1);
        if (verbose > 3) {
          printf("Buffer after read 1:\n");
          for(ii = 0; ii < previously_read; ii++) {
              printf("ii = %d, ", (int)ii);
              printf("data[ii] = %X\n", (int)data[ii]);
          }
        }

        if (buffer_position > previously_read) {
            buffer_position = buffer_position - BUFFER_SIZE;
        }
        device_id = data[buffer_position];

        if (verbose > 1) {
            printf("device id: %d (hex=%x)\n", (int)device_id, (int)device_id);
        }

        frame_size = data[buffer_position + 1];

        if (verbose > 1) {
            printf("frame size: %d (hex=%x)\n", (int)frame_size, (int)frame_size);
        }

        previously_read = readFromSource(1, data, buffer_position + 2, previously_read, frame_size);
        
        if (verbose > 3) {
          printf("Buffer after read frame:\n");
          for(ii = 0; ii < previously_read; ii++) {
              printf("ii = %d, ", (int)ii);
              printf("data[ii] = %X\n", (int)data[ii]);
          }
        }

        if (buffer_position > previously_read) {
            printf("array shifting happened\n");
            buffer_position = 0;//buffer_position - BUFFER_SIZE;
        }

        crc = data[buffer_position + frame_size + 1];
        uint8_t crc_fact = calculateCRC(data, buffer_position + 2, frame_size);

        if (crc_fact == crc) {
            processCrsfFrame(data + buffer_position + 2, device_id, frame_size, crc, crc_failures_count, &crsf_packet, &CRSF_LinkStatistics, &CRSF_Attitude);
            position = position + saveCurrentState(&data_for_flash_obj, position, &crsf_packet, &CRSF_LinkStatistics, &CRSF_Attitude);
            buffer_position = buffer_position + frame_size + 2;
            //previously_read = 0;

            //for(i = 0; i < BUFFER_SIZE; i++) {
            //    data[i] = 0;
            //}

            printf("crc ok, crc = %d, frame_size: %d, dump_position = %d (%X), buffer_position: %d, previously read: %d\n", crc, frame_size, 0, 0, buffer_position, previously_read);
            previously_read = readFromSource(1, data, buffer_position, previously_read, 1);
        } else {
            crc_failures_count++;
            // search for another beginning
            if (verbose > 1) {
                printf("CRC failed: crc fact: %d,  crc read: %d\n", crc_fact, crc);
                printf("frame_size: %d, dump_position = %d (%X), buffer_position: %d, previously read: %d\n", frame_size, 0, 0, buffer_position, previously_read);
            }
            uint8_t found = 0;

            for(uint16_t i = buffer_position + 1; i < buffer_position + frame_size; i++) {
                if (verbose > 2) {
                    printf("Searching for new beginning in buffer, i=%d, data[i]=%0X, dump position: %d\n", i, data[i], 0);
                }
                if (i > previously_read) {
                    buffer_position = 0;
                    previously_read = 0;

                    for(ii = 0; ii < BUFFER_SIZE; ii++) {
                        data[ii] = 0;
                    }

                    break;
                }

                if (data[i] == CRSF_SYNC_BYTE) {
                    buffer_position = i;
                    if (verbose > 1) {
                        printf("- Found!\n"); 
                    }
                    found = 1;
                    break;
                }
            }

            if (!found) {
                buffer_position = 0;

                do {
                    readFromSource(1, data, buffer_position, 0, 1);

                    if (verbose > 1) {
                        printf("Searching for new beginning in file, dump position: %X (%d), char: %x\n", 0, 0, data[buffer_position]);
                    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 23;
  RCC_OscInitStruct.PLL.PLLN = 354;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint32_t address_received = 0x08007000;
uint32_t address_transmitted = 0x08103a00;

int __io_putchar(int ch)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  //HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  // ITM_SendChar(ch);
  //CDC_Transmit_FS((uint8_t *)&ch, 1);

  return ch;
}

int _write(int file, char *ptr, int len) {
    return len;
    static uint8_t rc = USBD_OK;

    do {
        rc = CDC_Transmit_FS(ptr, len);
    } while (USBD_BUSY == rc);

    if (USBD_FAIL == rc) {
        /// NOTE: Should never reach here.
        /// TODO: Handle this error.
        return 0;
    }
    return len;
}

void store_received_data(char * cData) 
{
    uint32_t tick = 0;//HAL_GetTick();//__HAL_TIM_GET_COUNTER(&htim1);
    write_to_flash(t_received, address_received, tick, cData);
    t_received = t_received + BUFFER_SIZE;
}

void store_transmitted_data(char cData) {
    uint32_t tick = HAL_GetTick();//__HAL_TIM_GET_COUNTER(&htim1);
    write_to_flash(t_transmitted, address_transmitted, tick, &cData);
    t_transmitted++;
}

void handle_recieved_data(char cData) {

}

int write_to_flash(uint32_t t, uint32_t address_beginning, char * data, int item_size)
{
      /* Unlock the Flash to enable the flash control register access *************/
       HAL_FLASH_Unlock();

       /* Erase the user Flash area*/

      uint32_t flashAddress = address_beginning + t*item_size;
      uint64_t i;

      for(i = 0; i < item_size; i++, flashAddress++) {
          if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddress, (uint64_t) data[i]) != HAL_OK) {
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

    if (HAL_UART_Receive(&huart, (uint8_t*) data, n, 20000) == HAL_OK) {
        return n;
    }

    return -2;
}

int saveCurrentState(
  struct data_for_flash * data_for_flash_p,
  int position,
  struct crsfPacket_s * crsf_packet,
  struct crsfPayloadLinkstatistics_s * crsf_link_statistics, 
  struct crsfPayloadAttitude_s * crsf_attitude
) {
  int dataChanged = 0;

  if (data_for_flash_p->rx_pitch != crsf_packet->received_channels[0])
  {
     dataChanged = 1;
     data_for_flash_p->rx_pitch = crsf_packet->received_channels[0];
  }

  if (data_for_flash_p->rx_roll != crsf_packet->received_channels[1])
  {
     dataChanged = 1;
     data_for_flash_p->rx_roll = crsf_packet->received_channels[1];
  }

  if (data_for_flash_p->rx_yaw != crsf_packet->received_channels[2])
  {
     dataChanged = 1;
     data_for_flash_p->rx_yaw = crsf_packet->received_channels[2];
  }

  if (data_for_flash_p->rx_trottle != crsf_packet->received_channels[3])
  {
     dataChanged = 1;
     data_for_flash_p->rx_trottle = crsf_packet->received_channels[3];
  }

  if (data_for_flash_p->rx_arm != crsf_packet->received_channels[4])
  {
     dataChanged = 1;
     data_for_flash_p->rx_arm = crsf_packet->received_channels[4];
  }

  if (data_for_flash_p->pitch != crsf_attitude->pitch)
  {
     dataChanged = 1;
     data_for_flash_p->pitch = crsf_attitude->pitch;
  }

  if (data_for_flash_p->roll != crsf_attitude->roll)
  {
     dataChanged = 1;
     data_for_flash_p->roll = crsf_attitude->roll;
  }

  if (data_for_flash_p->yaw != crsf_attitude->yaw)
  {
     dataChanged = 1;
     data_for_flash_p->yaw = crsf_attitude->yaw;
  }

  if (data_for_flash_p->uplink_quality != crsf_link_statistics->uplink_Link_quality)
  {
     dataChanged = 1;
     data_for_flash_p->uplink_quality = crsf_link_statistics->uplink_Link_quality;
  }

    if (data_for_flash_p->downlink_quality != crsf_link_statistics->downlink_Link_quality)
  {
     dataChanged = 1;
     data_for_flash_p->downlink_quality = crsf_link_statistics->downlink_Link_quality;
  }

  if (dataChanged) {
    data_for_flash_p->tick = (uint64_t) HAL_GetTick();
    data_for_flash_p->beginning_marker = 0xee;
    write_to_flash(position, address_received, (char *)data_for_flash_p, sizeof(struct data_for_flash));
    printf("Position saved, position: %d, size: %d, \n", (int)position, (int)sizeof(struct data_for_flash));
    return 1;
  } else {
    printf("Position is the same. No save.");
    return 0;
  }

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
