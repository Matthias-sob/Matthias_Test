/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed))
{
    uint32_t time_stamp;
    uint16_t index;
    uint16_t sens_status;

    float accel_x;
    float accel_y;
    float accel_z;

    float rate_x;
    float rate_y;
    float rate_z;

    uint16_t temperature;
} ImuSensorData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

UART_HandleTypeDef huart4;
DMA_NodeTypeDef Node_GPDMA1_Channel7;
DMA_QListTypeDef List_GPDMA1_Channel7;
DMA_HandleTypeDef handle_GPDMA1_Channel7;

/* USER CODE BEGIN PV */
// ###############################################################################
uint8_t rx_byte;

static uint8_t rxBuf[129];

volatile uint32_t rx_total_bytes = 0;
volatile uint32_t rx_last_size = 0;

// Variable für Status von Callback
volatile HAL_StatusTypeDef rx_restart_status = HAL_OK;

// Ringbuffer Variablen
uint8_t rb[4096];
volatile uint16_t rb_wr; // write Index
volatile uint16_t rb_rd; // read Index
volatile uint32_t rb_overruns; // zählt überläufe


// Für DMA Zugriff
#define DMA_RX_BUF_SIZE 1024
uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];
volatile uint16_t dma_last_pos = 0;

// Parser Varibalen
volatile uint8_t parser_msg_id = 0;
volatile uint16_t parser_length = 0;
volatile uint8_t parser_len_lsb = 0;
volatile uint8_t parser_len_msb = 0;

uint8_t parser_payload[256];
volatile uint16_t parser_payload_index = 0;

volatile uint16_t parser_crc = 0;
volatile uint8_t parser_etx = 0;

volatile uint32_t parser_frame_count = 0;

// Error counter
volatile uint32_t parser_crc_error_count = 0;
volatile uint32_t parser_etx_error_count = 0;
volatile uint32_t parser_length_error_count = 0;
volatile uint32_t parser_frame_error_count = 0;

// Variablen für CRC Buffer
uint8_t crc_input_buffer[259];



typedef enum
{
    PARSER_WAIT_SYNC1 = 0,
    PARSER_WAIT_SYNC2,
    PARSER_READ_MSG_ID,
    PARSER_READ_LEN_LSB,
    PARSER_READ_LEN_MSB,
    PARSER_READ_PAYLOAD,
    PARSER_READ_CRC_LSB,
    PARSER_READ_CRC_MSB,
    PARSER_READ_ETX
} parser_state_t;

// für Parser
volatile parser_state_t parser_state = PARSER_WAIT_SYNC1;
volatile uint32_t parser_header_count = 0;

volatile uint32_t parser_imu_print_count = 0;

//###############################################################################
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_UART4_Init(void);
static uint16_t imu_calculate_crc(const uint8_t *data, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RB_SIZE 4096u
#define RB_MASK (RB_SIZE - 1u)


// CRC Check Funktion

static uint16_t imu_calculate_crc(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0u;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8u; bit++)
        {
            if (crc & 0x0001u)
            {
                crc >>= 1;
                crc ^= 0x8408u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}


// Funktion, die in Buffer schreibt

static inline void rb_write_bytes(const uint8_t *data, uint16_t len)
{
	// Byte für Byte in Ringpuffer schreiben
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t next_wr = (uint16_t)((rb_wr + 1u) & RB_MASK);

        // Voll? -> ältestes Byte verwerfen (read-pointer vorschieben)
        if (next_wr == rb_rd)
        {
            rb_rd = (uint16_t)((rb_rd + 1u) & RB_MASK);
            rb_overruns++;
        }

        rb[rb_wr] = data[i];
        rb_wr = next_wr;
    }
}

// Funktion, wie viele Bytes zur Verfügung stehen.
static inline uint16_t rb_available(void)
{
    return (uint16_t)((rb_wr - rb_rd) & RB_MASK);
}

// Position des DMA Zeigers auslesen
static inline uint16_t dma_get_pos(void)
{
    return (uint16_t)(DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx));
}

// Hilfsfunktion um IMU Daten auszugeben

static void print_imu_data(const ImuSensorData_t *imu)
{
    printf("ts=%lu idx=%u status=%u ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f T=%u\r\n",
           (unsigned long)imu->time_stamp,
           (unsigned int)imu->index,
           (unsigned int)imu->sens_status,
           (double)imu->accel_x,
           (double)imu->accel_y,
           (double)imu->accel_z,
           (double)imu->rate_x,
           (double)imu->rate_y,
           (double)imu->rate_z,
           (unsigned int)imu->temperature);
}

static void imu_parser_feed(uint8_t byte)
{
	//printf("%02X ", byte);
    switch (parser_state)
    {
        case PARSER_WAIT_SYNC1:
            if (byte == 0xFF)
            {
                parser_state = PARSER_WAIT_SYNC2;
            }
            break;

        case PARSER_WAIT_SYNC2:
            if (byte == 0x5A)
            {
                parser_header_count++;
                parser_state = PARSER_READ_MSG_ID;
            }
            else if (byte == 0xFF)
            {
                parser_state = PARSER_WAIT_SYNC2;
            }
            else
            {
                parser_state = PARSER_WAIT_SYNC1;
            }
            break;

        case PARSER_READ_MSG_ID:
            parser_msg_id = byte;
            parser_state = PARSER_READ_LEN_LSB;
            break;

        case PARSER_READ_LEN_LSB:
            parser_len_lsb = byte;
            parser_state = PARSER_READ_LEN_MSB;
            break;

        case PARSER_READ_LEN_MSB:
            parser_len_msb = byte;
        	parser_length = (uint16_t)parser_len_lsb | ((uint16_t)byte << 8);

            if (parser_length > sizeof(parser_payload))
            {
            	parser_length_error_count++;
                parser_state = PARSER_WAIT_SYNC1;
            }
            else if (parser_length == 0)
            {
                parser_payload_index = 0;
                parser_state = PARSER_READ_CRC_LSB;
            }
            else
            {
                parser_payload_index = 0;
                parser_state = PARSER_READ_PAYLOAD;
            }
            break;

        case PARSER_READ_PAYLOAD:
            parser_payload[parser_payload_index++] = byte;

            if (parser_payload_index >= parser_length)
            {
                parser_state = PARSER_READ_CRC_LSB;
            }
            break;

        case PARSER_READ_CRC_LSB:
            parser_crc = byte;
            parser_state = PARSER_READ_CRC_MSB;
            break;

        case PARSER_READ_CRC_MSB:
            parser_crc |= ((uint16_t)byte << 8);
            parser_state = PARSER_READ_ETX;
            break;

        case PARSER_READ_ETX:
            parser_etx = byte;

            if (parser_etx == 0x03)
            {
                crc_input_buffer[0] = parser_msg_id;
                crc_input_buffer[1] = parser_len_lsb;
                crc_input_buffer[2] = parser_len_msb;
                memcpy(&crc_input_buffer[3], parser_payload, parser_length);

                uint16_t calculated_crc = imu_calculate_crc(crc_input_buffer, (uint16_t)(3u + parser_length));

               // printf("CRC vergleich: calc=%04X recv=%04X ",  calculated_crc, parser_crc);

                if (calculated_crc == parser_crc)
                {
                    parser_frame_count++;

                    if ((parser_msg_id == 2u) && (parser_length == sizeof(ImuSensorData_t)))
                    {
                        const ImuSensorData_t *imu = (const ImuSensorData_t *)parser_payload;

                        parser_imu_print_count++;

                        if ((parser_imu_print_count % 5u) == 0u)
                        {
                            print_imu_data(imu);
                        }
                    }
                }
                else
                {
                    parser_crc_error_count++;
                    printf("CRC ERROR: calc=%04X recv=%04X id=%02X len=%u\r\n",
                           calculated_crc,
                           parser_crc,
                           parser_msg_id,
                           parser_length);
                }
            }
            else
            {
                parser_frame_error_count++;
            }

            parser_state = PARSER_WAIT_SYNC1;
            break;

        default:
            parser_state = PARSER_WAIT_SYNC1;
            break;
    }
}

static void process_new_dma_data(void)
{
    uint16_t dma_pos = dma_get_pos();

    if (dma_pos == dma_last_pos)
        return;

    if (dma_pos > dma_last_pos)
    {
        // einfacher Fall (kein Wrap)
        for (uint16_t i = dma_last_pos; i < dma_pos; i++)
        {
            uint8_t byte = dma_rx_buf[i];

            // Testausgabe
            imu_parser_feed(byte);
        }
    }
    else
    {
        // Wrap-Around Fall

        // Bereich 1: letzter Teil des Buffers
        for (uint16_t i = dma_last_pos; i < DMA_RX_BUF_SIZE; i++)
        {
            uint8_t byte = dma_rx_buf[i];
            imu_parser_feed(byte);
        }

        // Bereich 2: Anfang des Buffers
        for (uint16_t i = 0; i < dma_pos; i++)
        {
            uint8_t byte = dma_rx_buf[i];
            imu_parser_feed(byte);
        }
    }



    dma_last_pos = dma_pos;
}





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
  MX_GPDMA1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
  //HAL_UARTEx_ReceiveToIdle_IT(&huart4, rxBuf, sizeof(rxBuf));


  if (HAL_UART_Receive_DMA(&huart4, dma_rx_buf, DMA_RX_BUF_SIZE) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  process_new_dma_data();

	     printf("headers=%lu frames=%lu error crc=%lu error etx=%lu error length=%lu er\r\n",
	            (unsigned long)parser_header_count,
	            (unsigned long)parser_frame_count,
	            (unsigned long)parser_crc_error_count, parser_etx_error_count, parser_frame_error_count );

/*

	  uint16_t dma_pos = dma_get_pos();

	  if (dma_pos != dma_last_pos)
	  {
	      printf("new data from %u to %u\r\n",
	             dma_last_pos,
	             dma_pos);

	      dma_last_pos = dma_pos;
	  }

	  */

	  /*


	  printf("dma_pos: %u | dma_last_pos: %u\r\n",
	         (unsigned int)dma_get_pos(),
	         (unsigned int)dma_last_pos);

	         */

	    BSP_LED_Toggle(LED_GREEN);
	    HAL_Delay(500);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 250;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel7_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 256000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UCPD_CC1_Pin UCPD_CC2_Pin */
  GPIO_InitStruct.Pin = UCPD_CC1_Pin|UCPD_CC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_N_Pin USB_FS_P_Pin */
  GPIO_InitStruct.Pin = USB_FS_N_Pin|USB_FS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXT_EN_Pin RMI_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TXT_EN_Pin|RMI_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_TX_Pin ARD_D0_RX_Pin */
  GPIO_InitStruct.Pin = ARD_D1_TX_Pin|ARD_D0_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART4)
    {
        rx_total_bytes += Size;
        rx_last_size = Size;

        // Bytes in Ringbuffer schreiben
        rb_write_bytes(rxBuf, Size);

        //printf("Empfangen: %s\r\n", rxBuf);
        BSP_LED_Toggle(LED_YELLOW);

        rx_restart_status = HAL_UARTEx_ReceiveToIdle_IT(&huart4, rxBuf, sizeof(rxBuf));
    }
}



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == UART4)
//    {
//
//    	//HAL_UART_Transmit(&hcom_uart, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);
//        BSP_LED_Toggle(LED_YELLOW);
//        printf("Empfangen: %c\r\n", rx_byte);
//
//        // Empfang wieder neu starten!
//        HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
//
//    }
//}
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
#ifdef USE_FULL_ASSERT
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
