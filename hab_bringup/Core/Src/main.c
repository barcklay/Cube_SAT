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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* BMP390 calibration coefficients, already scaled to floating point (datasheet 8.4). */
typedef struct
{
  double t1, t2, t3;
  double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
} BmpCalib;

/* One flash page (256 bytes) that survives reset and power-off. */
typedef struct
{
  uint32_t magic;
  uint32_t boot_count;
  uint8_t pattern[248];
} BootRecord;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP390_ADDR      (0x77U << 1)
#define BMP_REG_CHIP_ID  0x00U
#define BMP_REG_ERR      0x02U
#define BMP_REG_DATA     0x04U  /* 6 bytes: pressure xlsb..msb, temperature xlsb..msb */
#define BMP_REG_PWR_CTRL 0x1BU
#define BMP_REG_OSR      0x1CU
#define BMP_REG_ODR      0x1DU
#define BMP_REG_CONFIG   0x1FU
#define BMP_REG_CALIB    0x31U  /* 21 bytes of calibration data */
#define BMP_REG_CMD      0x7EU

#define FLASH_CMD_WREN   0x06U
#define FLASH_CMD_RDSR   0x05U
#define FLASH_CMD_READ   0x03U
#define FLASH_CMD_PP     0x02U  /* page program, up to 256 bytes */
#define FLASH_CMD_SE     0x20U  /* 4 KB sector erase */
#define FLASH_CMD_JEDEC  0x9FU

#define FLASH_BOOT_ADDR  0x000000U  /* sector 0: boot record */
#define FLASH_DEMO_ADDR  0x001000U  /* sector 1: "write without erase" demo */
#define BOOT_MAGIC       0xCAFE5A7EU

#define STD_SEA_LEVEL_PA 101325.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static BmpCalib bmp_cal;
static double ground_pa;  /* pressure at power-on, reference for relative altitude */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* printf here is newlib-nano without %f, so print fixed-point: 2 decimals. */
static void PrintFixed2(double v)
{
  long x = lround(v * 100.0);
  if (x < 0)
  {
    printf("-");
    x = -x;
  }
  printf("%ld.%02ld", x / 100, x % 100);
}

/* ---------------- BMP390 (I2C) ---------------- */

static HAL_StatusTypeDef BmpRead(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, BMP390_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

static HAL_StatusTypeDef BmpWrite(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c1, BMP390_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static int BmpInit(void)
{
  uint8_t id = 0;
  if (BmpRead(BMP_REG_CHIP_ID, &id, 1) != HAL_OK || id != 0x60U)
  {
    printf("BMP390: not found (id=0x%02X)\r\n", (unsigned int)id);
    return -1;
  }
  printf("BMP390 chip ID: 0x%02X\r\n", (unsigned int)id);

  BmpWrite(BMP_REG_CMD, 0xB6U);  /* soft reset */
  HAL_Delay(10);

  /* Raw calibration from NVM, then scale each value as in datasheet 8.4. */
  uint8_t c[21];
  if (BmpRead(BMP_REG_CALIB, c, sizeof(c)) != HAL_OK)
  {
    printf("BMP390: calibration read failed\r\n");
    return -1;
  }
  bmp_cal.t1 = ldexp((uint16_t)(c[1] << 8 | c[0]), 8);
  bmp_cal.t2 = ldexp((uint16_t)(c[3] << 8 | c[2]), -30);
  bmp_cal.t3 = ldexp((int8_t)c[4], -48);
  bmp_cal.p1 = ldexp((int16_t)(c[6] << 8 | c[5]) - 16384, -20);
  bmp_cal.p2 = ldexp((int16_t)(c[8] << 8 | c[7]) - 16384, -29);
  bmp_cal.p3 = ldexp((int8_t)c[9], -32);
  bmp_cal.p4 = ldexp((int8_t)c[10], -37);
  bmp_cal.p5 = ldexp((uint16_t)(c[12] << 8 | c[11]), 3);
  bmp_cal.p6 = ldexp((uint16_t)(c[14] << 8 | c[13]), -6);
  bmp_cal.p7 = ldexp((int8_t)c[15], -8);
  bmp_cal.p8 = ldexp((int8_t)c[16], -15);
  bmp_cal.p9 = ldexp((int16_t)(c[18] << 8 | c[17]), -48);
  bmp_cal.p10 = ldexp((int8_t)c[19], -48);
  bmp_cal.p11 = ldexp((int8_t)c[20], -65);

  BmpWrite(BMP_REG_OSR, 0x03U);       /* pressure x8, temperature x1 */
  BmpWrite(BMP_REG_ODR, 0x04U);       /* new sample every 80 ms (12.5 Hz) */
  BmpWrite(BMP_REG_CONFIG, 0x04U);    /* IIR filter coefficient 3: smooths noise */
  BmpWrite(BMP_REG_PWR_CTRL, 0x33U);  /* pressure on, temperature on, normal mode */
  HAL_Delay(100);

  uint8_t err = 0;
  BmpRead(BMP_REG_ERR, &err, 1);
  if (err != 0)
  {
    printf("BMP390: config error 0x%02X\r\n", (unsigned int)err);
    return -1;
  }
  return 0;
}

/* Raw ADC values -> degrees C and Pa (datasheet 8.5, 8.6). */
static int BmpReadData(double *temp_c, double *press_pa)
{
  uint8_t d[6];
  if (BmpRead(BMP_REG_DATA, d, sizeof(d)) != HAL_OK)
  {
    return -1;
  }
  double up = (double)((uint32_t)d[2] << 16 | (uint32_t)d[1] << 8 | d[0]);
  double ut = (double)((uint32_t)d[5] << 16 | (uint32_t)d[4] << 8 | d[3]);

  double pd1 = ut - bmp_cal.t1;
  double t = pd1 * bmp_cal.t2 + pd1 * pd1 * bmp_cal.t3;

  double out1 = bmp_cal.p5 + bmp_cal.p6 * t + bmp_cal.p7 * t * t + bmp_cal.p8 * t * t * t;
  double out2 = up * (bmp_cal.p1 + bmp_cal.p2 * t + bmp_cal.p3 * t * t + bmp_cal.p4 * t * t * t);
  double out3 = up * up * (bmp_cal.p9 + bmp_cal.p10 * t) + up * up * up * bmp_cal.p11;

  *temp_c = t;
  *press_pa = out1 + out2 + out3;
  return 0;
}

/* Barometric formula (standard atmosphere), valid up to ~11 km. */
static double PressureToAltitude(double press_pa, double ref_pa)
{
  return 44330.0 * (1.0 - pow(press_pa / ref_pa, 1.0 / 5.255));
}

/* ---------------- SPI flash (GD25Q128 / W25Q128) ---------------- */

static void FlashSelect(void)
{
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

static void FlashDeselect(void)
{
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

/* Command byte + 24-bit address, most significant byte first. */
static void FlashSendCmdAddr(uint8_t cmd, uint32_t addr)
{
  uint8_t hdr[4] = {cmd, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr};
  HAL_SPI_Transmit(&hspi1, hdr, sizeof(hdr), 100);
}

static void FlashWriteEnable(void)
{
  uint8_t cmd = FLASH_CMD_WREN;
  FlashSelect();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
  FlashDeselect();
}

/* Status register bit 0 (BUSY) is 1 while erase/program is running. */
static int FlashWaitReady(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint8_t cmd = FLASH_CMD_RDSR;
  uint8_t sr = 0;
  do
  {
    FlashSelect();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, &sr, 1, 100);
    FlashDeselect();
    if ((sr & 0x01U) == 0)
    {
      return 0;
    }
  } while (HAL_GetTick() - start < timeout_ms);
  return -1;
}

static void FlashReadJedec(uint8_t id[3])
{
  uint8_t cmd = FLASH_CMD_JEDEC;
  FlashSelect();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
  HAL_SPI_Receive(&hspi1, id, 3, 100);
  FlashDeselect();
}

static void FlashRead(uint32_t addr, uint8_t *buf, uint16_t len)
{
  FlashSelect();
  FlashSendCmdAddr(FLASH_CMD_READ, addr);
  HAL_SPI_Receive(&hspi1, buf, len, 1000);
  FlashDeselect();
}

/* Sets every bit of a 4 KB sector back to 1 (all bytes = 0xFF). */
static int FlashEraseSector(uint32_t addr)
{
  FlashWriteEnable();
  FlashSelect();
  FlashSendCmdAddr(FLASH_CMD_SE, addr);
  FlashDeselect();
  return FlashWaitReady(1000);
}

/* Programming can only turn bits 1 -> 0, never 0 -> 1. Must stay inside one 256-byte page. */
static int FlashProgramPage(uint32_t addr, const uint8_t *data, uint16_t len)
{
  FlashWriteEnable();
  FlashSelect();
  FlashSendCmdAddr(FLASH_CMD_PP, addr);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 100);
  FlashDeselect();
  return FlashWaitReady(100);
}

/* Boot counter in sector 0: proves data survives reset and power-off. */
static void FlashBootTest(void)
{
  uint8_t id[3];
  FlashReadJedec(id);
  printf("Flash JEDEC: %02X %02X %02X\r\n", id[0], id[1], id[2]);

  BootRecord rec;
  FlashRead(FLASH_BOOT_ADDR, (uint8_t *)&rec, sizeof(rec));

  uint32_t prev = 0;
  if (rec.magic == BOOT_MAGIC)
  {
    int intact = 1;
    for (uint32_t i = 0; i < sizeof(rec.pattern); i++)
    {
      if (rec.pattern[i] != (uint8_t)(i + rec.boot_count))
      {
        intact = 0;
      }
    }
    prev = rec.boot_count;
    printf("Flash: found record from previous boot, boot_count=%lu, data %s\r\n",
           (unsigned long)prev, intact ? "intact" : "CORRUPTED");
  }
  else
  {
    printf("Flash: no record yet (first boot or blank sector)\r\n");
  }

  rec.magic = BOOT_MAGIC;
  rec.boot_count = prev + 1;
  for (uint32_t i = 0; i < sizeof(rec.pattern); i++)
  {
    rec.pattern[i] = (uint8_t)(i + rec.boot_count);
  }

  BootRecord check;
  int ok = FlashEraseSector(FLASH_BOOT_ADDR) == 0
        && FlashProgramPage(FLASH_BOOT_ADDR, (uint8_t *)&rec, sizeof(rec)) == 0;
  FlashRead(FLASH_BOOT_ADDR, (uint8_t *)&check, sizeof(check));
  ok = ok && memcmp(&rec, &check, sizeof(rec)) == 0;
  printf("Flash: erase + write + read-back %s, boot_count now %lu\r\n",
         ok ? "OK" : "FAILED", (unsigned long)rec.boot_count);
}

/* Why erase is required: programming only clears bits, so old & new = result. */
static void FlashNoEraseDemo(void)
{
  uint8_t b;
  FlashEraseSector(FLASH_DEMO_ADDR);
  FlashRead(FLASH_DEMO_ADDR, &b, 1);
  printf("Flash demo: after erase          -> %02X\r\n", b);
  b = 0xF0U;
  FlashProgramPage(FLASH_DEMO_ADDR, &b, 1);
  FlashRead(FLASH_DEMO_ADDR, &b, 1);
  printf("Flash demo: write F0             -> %02X\r\n", b);
  b = 0x0FU;
  FlashProgramPage(FLASH_DEMO_ADDR, &b, 1);
  FlashRead(FLASH_DEMO_ADDR, &b, 1);
  printf("Flash demo: write 0F, no erase   -> %02X (F0 & 0F)\r\n", b);
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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  FlashDeselect();
  HAL_Delay(100);
  printf("\r\n=== hab_bringup: BMP390 + SPI flash ===\r\n");

  FlashBootTest();
  FlashNoEraseDemo();

  int bmp_ok = BmpInit() == 0;
  if (bmp_ok)
  {
    /* Average 10 samples at power-on as the "ground" reference. */
    double sum = 0.0;
    double t, p;
    for (int i = 0; i < 10; i++)
    {
      HAL_Delay(100);
      BmpReadData(&t, &p);
      sum += p;
    }
    ground_pa = sum / 10.0;
    printf("Ground reference: ");
    PrintFixed2(ground_pa);
    printf(" Pa\r\n");
  }
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    double temp_c, press_pa;
    if (bmp_ok && BmpReadData(&temp_c, &press_pa) == 0)
    {
      printf("T=");
      PrintFixed2(temp_c);
      printf(" C  P=");
      PrintFixed2(press_pa);
      printf(" Pa  alt=");
      PrintFixed2(PressureToAltitude(press_pa, STD_SEA_LEVEL_PA));
      printf(" m  rel=");
      PrintFixed2(PressureToAltitude(press_pa, ground_pa));
      printf(" m\r\n");
    }
    else
    {
      printf("BMP390 read failed\r\n");
    }
    HAL_Delay(500);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Retarget printf (syscalls.c _write -> __io_putchar) to LPUART1 / ST-LINK VCP */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&hlpuart1, &c, 1, HAL_MAX_DELAY);
  return ch;
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
