/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AmazingHand SCS bus passthrough firmware
  ******************************************************************************
  * @attention
  *
  * AmazingHand dorm test bench (see host/app/control/backend_mcu.py for the
  * PC-side peer and tools/mcu_firmware/README.md for wiring).
  *
  * Architecture:
  *   PC (AmazingHand host, pyserial)  --USART2 @115200 (ST-Link VCP)--> this
  *   board --USART1 PA9 single-wire half-duplex @1 Mbps--> 8 x SCS0009 servos
  *   (Feetech SCS-Y4 distribution boards, tree-wired to one 3-pin bus).
  *
  * Protocol (frame-length prefixed, binary):
  *   PC -> board: [len u8][len bytes]   len = 1..64, a raw SCS request frame
  *   board -> PC: [len u8][len bytes]   len = 0 means "no response"
  *   The SCS frame bytes themselves are built by the PC golden model
  *   (host/app/control/scs_protocol.py); this firmware is a byte pump only.
  *
  * Half-duplex handling: USART1 runs with HDSEL set, so every transmitted
  * byte is looped back on the same pin. The echo (len bytes) is drained and
  * discarded before collecting the servo response (3 ms bus silence ends the
  * response window).
  *
  * Wiring (mandatory): servo 5 V from an INDEPENDENT supply (never from this
  * board or USB); STM32 GND must be common with the servo supply GND.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AH_FRAME_MAX 64U        /* one SCS frame, largest is SYNC_WRITE 32 B */
#define AH_RESP_MAX 80U         /* response: FF FF ID LEN ERR 8B CS = 13 B max */
#define AH_RESP_SILENCE_MS 3U   /* bus idle time that ends a response window */
#define AH_RESP_WINDOW_MS 20U   /* overall response collection window */
#define AH_RESP_FIRST_MS 2U     /* no first byte in this time => no response */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static UART_HandleTypeDef huart1;  /* SCS servo bus, PA9, 1 Mbps half-duplex */
static uint8_t ah_frame[AH_FRAME_MAX];
static uint8_t ah_resp[AH_RESP_MAX];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SCS_Bus_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief USART1 init, written by hand (not in CubeMX .ioc on purpose).
 *
 * PA9 AF push-pull, 1 Mbps (APB2 = 72 MHz -> BRR = 72, no fraction),
 * half-duplex single-wire (HDSEL). CubeMX does not own USART1 in this
 * firmware; this bypass is allowed by the repo rules.
 */
static void SCS_Bus_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000U;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;   /* HDSEL collapses it to one wire */
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  SET_BIT(huart1.Instance->CR3, USART_CR3_HDSEL);
}

/**
 * @brief Drain the half-duplex loopback echo of a just-sent frame.
 *
 * HDSEL loops transmitted bytes back into RX. Each byte takes 10 bit times
 * at 1 Mbps (10 us); the echo drains while the servos keep their lines
 * released. Any leftover RX noise is flushed here as well.
 */
static void SCS_DrainEcho(uint8_t len)
{
  uint8_t b;
  uint8_t dropped = 0;
  uint32_t t0 = HAL_GetTick();
  while (dropped < len && (HAL_GetTick() - t0) < 5U)
  {
    if (HAL_UART_Receive(&huart1, &b, 1U, 1U) != HAL_OK)
    {
      break;  /* no echo / not HDSEL: do not stall the frame */
    }
    dropped++;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  SCS_Bus_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for (;;)
  {
    uint8_t len = 0;
    if (HAL_UART_Receive(&huart2, &len, 1U, HAL_MAX_DELAY) != HAL_OK)
    {
      continue;
    }
    if (len == 0U || len > AH_FRAME_MAX)
    {
      /* Protocol ACK for an invalid length: reply [0] = "no response". */
      uint8_t zero = 0U;
      (void)HAL_UART_Transmit(&huart2, &zero, 1U, 100U);
      continue;
    }
    if (HAL_UART_Receive(&huart2, ah_frame, len, 100U) != HAL_OK)
    {
      uint8_t zero = 0U;
      (void)HAL_UART_Transmit(&huart2, &zero, 1U, 100U);
      continue;
    }

    /* Pump the raw SCS frame onto the servo bus. */
    (void)HAL_UART_Transmit(&huart1, ah_frame, len, 100U);
    SCS_DrainEcho(len);

    /* Collect the servo response until 3 ms of bus silence. */
    uint16_t rn = 0;
    uint32_t t0 = HAL_GetTick();
    uint32_t last = t0;
    while ((HAL_GetTick() - t0) < AH_RESP_WINDOW_MS && rn < AH_RESP_MAX)
    {
      uint8_t b;
      if (HAL_UART_Receive(&huart1, &b, 1U, 1U) == HAL_OK)
      {
        ah_resp[rn++] = b;
        last = HAL_GetTick();
      }
      else if (rn == 0U && (HAL_GetTick() - t0) >= AH_RESP_FIRST_MS)
      {
        break;  /* broadcast/SYNC_WRITE has no reply: return quickly */
      }
      if (rn > 0 && (HAL_GetTick() - last) > AH_RESP_SILENCE_MS)
      {
        break;
      }
    }

    /* Reply [len][bytes]; [0] when the bus stayed silent (offline servo). */
    uint8_t rl = (uint8_t)rn;
    (void)HAL_UART_Transmit(&huart2, &rl, 1U, 100U);
    if (rn > 0)
    {
      (void)HAL_UART_Transmit(&huart2, ah_resp, rn, 100U);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
