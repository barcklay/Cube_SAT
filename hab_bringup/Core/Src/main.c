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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "flight_sm.h"
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

/* Latest GPS data, filled from NMEA GGA and GSV sentences. */
typedef struct
{
  int fix_quality;  /* 0 = no fix, 1 = GPS fix, 2 = DGPS */
  int sats_used;
  double lat_deg;   /* + north */
  double lon_deg;   /* + east */
  double alt_m;     /* above mean sea level */
  char utc[7];      /* "hhmmss" */
} GpsData;

/* Last complete UBX frame from the GPS: answers to our config commands. */
typedef struct
{
  uint8_t cls;
  uint8_t id;
  uint16_t len;
  uint8_t payload[64];
  int ready;
} UbxFrame;

/* One telemetry frame: everything the flight computer knows at one moment.
   Filled every FRAME_PERIOD_MS; later written to flash (HW-16) and sent by radio (HW-13). */
typedef struct
{
  uint32_t seq;         /* frame number since boot */
  uint32_t t_ms;        /* time since boot when the frame was taken */
  uint16_t valid;       /* FRAME_OK_* bits: which sensors answered */
  float baro_pa;
  float baro_temp_c;
  float baro_rel_m;     /* altitude relative to the pressure at power-on */
  float acc_g[3];
  float gyro_dps[3];
  float out_temp_c;     /* DS18B20, refreshed once per second */
  uint8_t gps_fix;
  uint8_t gps_sats;
  uint8_t gps_in_view;
  int32_t gps_lat_e7;   /* degrees * 1e7, like u-blox */
  int32_t gps_lon_e7;
  float gps_alt_m;
  uint32_t work_us;     /* time spent reading sensors in this cycle */
  float alt_m;          /* altitude above the launch point, input of the state machine */
  float vz_mps;         /* vertical speed over the last 5 s, + = up */
  float asl_m;          /* altitude above sea level (barometer, reference: asl_src) */
  uint8_t state;        /* FlightState: PRELAUNCH ... LANDED (HW-18) */
  uint8_t asl_src;      /* ASL_STD / ASL_GPS / ASL_QNH */
} TelemetryFrame;

/* One log record in the SPI flash: exactly half a 256-byte page, so records never
   straddle a page. Erased flash reads 0xFF, so an unused slot has magic 0xFFFF. */
typedef struct
{
  uint16_t magic;        /* LOG_MAGIC */
  uint16_t boot;         /* boot number: shows where the board restarted */
  uint32_t index;        /* record number since the log was cleared = its slot number */
  TelemetryFrame frame;
  float ground_pa;       /* launch pressure: lets a reset in flight continue (HW-18) */
  float asl_offset;      /* sea-level reference at that time */
  float max_alt;         /* highest altitude above launch so far */
  uint8_t reset_cause;   /* why the board last restarted (RESET_*), 0xFF in older records */
  uint8_t reserved[128 - 8 - sizeof(TelemetryFrame) - 12 - 1 - 2];
  uint16_t crc;          /* CRC-16/CCITT of everything above */
} LogRecord;
_Static_assert(sizeof(LogRecord) == 128, "LogRecord must be 128 bytes");
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

#define GPS_RX_BUF_SIZE  2048U  /* ~2 s of NMEA at 9600 baud */
#define GPS_LINE_MAX     100U   /* NMEA sentences are at most 82 chars */
#define GPS_TALKERS      "PLABQ" /* GP GPS, GL GLONASS, GA Galileo, GB BeiDou, GQ QZSS */

#define UBX_CLASS_ACK    0x05U
#define UBX_ID_ACK       0x01U
#define UBX_ID_NAK       0x00U
#define UBX_CLASS_CFG    0x06U
#define UBX_ID_VALSET    0x8AU
#define UBX_ID_VALGET    0x8BU
#define UBX_LAYER_RAM    0x01U  /* VALSET layer bits */
#define UBX_LAYER_BBR    0x02U  /* battery-backed RAM: kept while the backup cell holds */
#define CFG_NAVSPG_DYNMODEL 0x20110021UL  /* key ID, 1-byte value */
#define DYNMODEL_AIRBORNE_4G 8U
/* ICM-42688-P IMU on its own SPI3 (PC10/PC11/PC12), CS on PC7 (D9). Until PC7 gets the IMU_CS label in CubeMX,
   the pin is set up by ImuCsInit() below. */
#ifndef IMU_CS_Pin
#define IMU_CS_Pin       GPIO_PIN_7
#define IMU_CS_GPIO_Port GPIOC
#define IMU_CS_MANUAL_INIT 1
#endif
#define IMU_REG_WHO_AM_I 0x75U
#define IMU_WHO_AM_I_VAL 0x47U
#define IMU_REG_INTF_CONFIG0 0x4CU  /* bits 1:0 UI_SIFS_CFG: 11 = disable I2C */
#define IMU_REG_TEMP_DATA1   0x1DU  /* 14 bytes: temp, accel X/Y/Z, gyro X/Y/Z, big-endian */
#define IMU_REG_PWR_MGMT0    0x4EU
#define IMU_ACCEL_LSB_PER_G  2048.0  /* default range +-16 g */
#define IMU_GYRO_LSB_PER_DPS 16.4    /* default range +-2000 deg/s */

/* DS18B20 1-Wire commands */
#define OW_CMD_READ_ROM     0x33U
#define OW_CMD_SKIP_ROM     0xCCU
#define OW_CMD_CONVERT_T    0x44U
#define OW_CMD_READ_SCRATCH 0xBEU

#define FRAME_PERIOD_MS  200U  /* 5 frames per second */
#define DS18B20_EVERY_N  5U    /* DS18B20 needs 750 ms per conversion: read it once per second */
#define LOOP_STATS_EVERY 25U   /* print loop timing statistics every 25 frames = 5 s */
#define FRAME_OK_BARO    0x01U
#define FRAME_OK_IMU     0x02U
#define FRAME_OK_OUT     0x04U
#define FRAME_OK_GPSFIX  0x08U

/* Flight log in the SPI flash: sectors 0-1 are used by the boot test, the log starts at 8 KB. */
#define LOG_MAGIC        0x484CU                 /* "LH": record with flight state (HW-18); older "LG" records are overwritten */
#define LOG_START_ADDR   0x002000UL
#define LOG_END_ADDR     0x1000000UL             /* 16 MB */
#define LOG_RECORD_SIZE  128U
#define SECTOR_SIZE      4096U
#define LOG_PER_SECTOR   (SECTOR_SIZE / LOG_RECORD_SIZE)  /* 32 */
#define LOG_SLOTS        ((LOG_END_ADDR - LOG_START_ADDR) / LOG_RECORD_SIZE)
#define LOG_QUEUE_LEN    16U                     /* frames waiting while the flash is busy */
#define FLASH_CMD_CHIP_ERASE 0xC7U

/* Independent watchdog: its own 32 kHz oscillator, /64 -> 500 Hz, reload 999 -> 2 s. */
#define IWDG_KEY_START   0xCCCCU
#define IWDG_KEY_ACCESS  0x5555U
#define IWDG_KEY_FEED    0xAAAAU
#define IWDG_PRESCALER_64 4U
#define IWDG_RELOAD_2S   999U

#define RESET_POWER      1U
#define RESET_PIN        2U  /* black RESET button or the programmer */
#define RESET_SOFTWARE   3U
#define RESET_WATCHDOG   4U
#define RESET_BROWNOUT   5U

#define FRAME_OK_GPSLINK 0x10U  /* GPS is sending data (fresh GGA in the last 3 s) */
#define GPS_STALE_MS     3000U
#define RETRY_EVERY_N    25U    /* re-try a lost sensor every 25 frames = 5 s */

#define WIRING_DIAG      1  /* 1 = at boot, probe the SPI flash wiring and print a verdict */
#define GPS_ECHO_NMEA    0  /* 1 = also print every valid NMEA sentence (for logging) */

/* Flight state machine (HW-18) */
#define FRAME_SIM        0x80U  /* frame from the 'S' simulation: the altitude is synthetic */
#define LANDED_LOG_EVERY 50U    /* after landing one record per 10 s: the flash lasts for days */
#define BUZZER_EVERY_N   15U    /* after landing: one 200 ms beep every 3 s */
#define SIM_SPEED        40.0   /* 'S' simulation runs 40x faster than a real flight */
#define ASL_STD          0U     /* sea level from the standard 1013.25 hPa */
#define ASL_GPS          1U     /* sea level calibrated against GPS altitude before launch */
#define ASL_QNH          2U     /* sea-level pressure typed in with 'Q' */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Init functions stay quiet when a lost sensor is being re-tried every few seconds. */
#define INIT_PRINTF(...) do { if (init_verbose) { printf(__VA_ARGS__); } } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */
static BmpCalib bmp_cal;
static double ground_pa;  /* pressure at power-on, reference for relative altitude */
static double asl_offset; /* altitude above sea level = IsaAltitude(p) + asl_offset */
static uint8_t asl_src = ASL_STD;
static FlightSm fsm;
static int sim_active;
static uint32_t sim_start_ms;
static uint32_t sim_rng = 12345U;
static LogRecord log_last;  /* last valid record found at boot: to continue after a reset */
static int log_last_ok;

/* Ring buffer: the UART interrupt writes at head, the main loop reads at tail. */
static uint8_t gps_rx_byte;
static uint8_t gps_rx_buf[GPS_RX_BUF_SIZE];
static volatile uint16_t gps_rx_head;
static volatile uint16_t gps_rx_tail;
static volatile uint32_t gps_rx_total;
static volatile uint32_t gps_rx_overflow;

static GpsData gps;
static UbxFrame ubx;
static int gsv_in_view[sizeof(GPS_TALKERS) - 1];  /* satellites in view per constellation */
static uint32_t nmea_ok, nmea_bad;
static int gps_boot_dynmodel = -1;
static uint16_t boot_id;
static uint8_t reset_cause;
static int init_verbose = 1;
static uint32_t gps_last_gga_ms;  /* tick of the last good GGA sentence */               /* boot counter from the boot record, stored in every log record */

/* Log state: next free slot, first slot that is not yet known to be erased, and a small
   queue for frames that arrive while the flash is busy erasing a sector. */
static uint32_t log_next;
static uint32_t log_erased_to;
static LogRecord log_queue[LOG_QUEUE_LEN];
static uint32_t log_q_head, log_q_tail;
static uint32_t log_written, log_dropped;
static int log_full;  /* RAM value right after power-up, before we set it */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* printf here is newlib-nano without %f, so print fixed-point with N decimals. */
static void PrintFixed(double v, int decimals)
{
  long scale = 1;
  for (int i = 0; i < decimals; i++)
  {
    scale *= 10;
  }
  long x = lround(v * (double)scale);
  if (x < 0)
  {
    printf("-");
    x = -x;
  }
  char frac[12];
  long rest = x % scale;
  for (int i = decimals - 1; i >= 0; i--)
  {
    frac[i] = (char)('0' + rest % 10);
    rest /= 10;
  }
  frac[decimals] = '\0';
  printf(decimals > 0 ? "%ld.%s" : "%ld", x / scale, frac);
}

static void PrintFixed2(double v)
{
  PrintFixed(v, 2);
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
    INIT_PRINTF("BMP390: not found (id=0x%02X)\r\n", (unsigned int)id);
    return -1;
  }
  INIT_PRINTF("BMP390 chip ID: 0x%02X\r\n", (unsigned int)id);

  BmpWrite(BMP_REG_CMD, 0xB6U);  /* soft reset */
  HAL_Delay(10);

  /* Raw calibration from NVM, then scale each value as in datasheet 8.4. */
  uint8_t c[21];
  if (BmpRead(BMP_REG_CALIB, c, sizeof(c)) != HAL_OK)
  {
    INIT_PRINTF("BMP390: calibration read failed\r\n");
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
    INIT_PRINTF("BMP390: config error 0x%02X\r\n", (unsigned int)err);
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

/* Altitude difference between two pressures in the layered standard atmosphere
   (flight_sm.h). The old single formula was ~4.7 km low at 30 km. */
static double PressureToAltitude(double press_pa, double ref_pa)
{
  return IsaAltitude(press_pa) - IsaAltitude(ref_pa);
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
  boot_id = (uint16_t)rec.boot_count;
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

/* ---------------- IMU ICM-42688-P (SPI3, own bus) ---------------- */

static void ImuCsInit(void)
{
#ifdef IMU_CS_MANUAL_INIT
  /* CS must idle high, otherwise the IMU answers while we talk to the flash. */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  GPIO_InitTypeDef g = {0};
  g.Pin = IMU_CS_Pin;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &g);
#endif
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

/* SPI read: first byte = register address with bit 7 set (= read). */
static uint8_t ImuReadReg(uint8_t reg)
{
  uint8_t tx = reg | 0x80U;
  uint8_t val = 0;
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &tx, 1, 100);
  HAL_SPI_Receive(&hspi3, &val, 1, 100);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return val;
}

static void ImuWriteReg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2] = {reg & 0x7FU, val};
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, tx, sizeof(tx), 100);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

/* The IMU shares its SPI pins with I2C (SCLK = SCL, SDI = SDA). We use SPI only,
   so switch the I2C interface off (it usually already is). */
static void ImuDisableI2c(void)
{
  uint8_t before = ImuReadReg(IMU_REG_INTF_CONFIG0);
  ImuWriteReg(IMU_REG_INTF_CONFIG0, (uint8_t)((before & ~0x03U) | 0x03U));
  uint8_t after = ImuReadReg(IMU_REG_INTF_CONFIG0);
  INIT_PRINTF("IMU INTF_CONFIG0: 0x%02X -> 0x%02X (I2C off)\r\n", (unsigned int)before, (unsigned int)after);
}

/* Burst read: the register address auto-increments while CS stays low. */
static void ImuReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint8_t tx = reg | 0x80U;
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &tx, 1, 100);
  HAL_SPI_Receive(&hspi3, buf, len, 100);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static int ImuInit(void)
{
  uint8_t id = ImuReadReg(IMU_REG_WHO_AM_I);
  INIT_PRINTF("IMU WHO_AM_I: 0x%02X (%s)\r\n", (unsigned int)id,
         id == IMU_WHO_AM_I_VAL ? "ICM-42688-P OK" : "unexpected");
  if (id != IMU_WHO_AM_I_VAL)
  {
    return -1;
  }
  ImuDisableI2c();
  ImuWriteReg(IMU_REG_PWR_MGMT0, 0x0FU);  /* gyro + accel on, low-noise mode */
  HAL_Delay(50);                          /* gyro needs ~45 ms to start */
  return 0;
}

/* Raw 16-bit big-endian values (after 2 temperature bytes) -> g and deg/s. */
static void ImuRead(float acc_g[3], float gyro_dps[3])
{
  uint8_t d[14];
  ImuReadRegs(IMU_REG_TEMP_DATA1, d, sizeof(d));
  for (int i = 0; i < 3; i++)
  {
    int16_t a = (int16_t)((uint16_t)d[2 + 2 * i] << 8 | d[3 + 2 * i]);
    int16_t g = (int16_t)((uint16_t)d[8 + 2 * i] << 8 | d[9 + 2 * i]);
    acc_g[i] = (float)(a / IMU_ACCEL_LSB_PER_G);
    gyro_dps[i] = (float)(g / IMU_GYRO_LSB_PER_DPS);
  }
}

/* ---------------- Wiring check for the SPI flash ----------------
   SPI1 is switched off and the pins are driven by hand (bit-bang), so we can try
   "what if" cases: CS on another socket, DI and DO swapped. The case where the flash
   answers its real JEDEC ID tells which wire is where. */

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  const char *name;
} DiagPin;

static void DiagPinMode(const DiagPin *p, uint32_t mode, uint32_t pull)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = p->pin;
  g.Mode = mode;
  g.Pull = pull;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(p->port, &g);
}

static void DiagDelay(void)
{
  for (volatile int i = 0; i < 40; i++)
  {
  }
}

/* SPI mode 0 by hand: set data, clock up (device samples), read, clock down. */
static void DiagTransfer(const DiagPin *cs, const DiagPin *mosi, const DiagPin *miso,
                         const DiagPin *sck, const uint8_t *tx, uint8_t *rx, int n)
{
  HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_RESET);
  DiagDelay();
  for (int i = 0; i < n; i++)
  {
    uint8_t in = 0;
    for (int bit = 7; bit >= 0; bit--)
    {
      HAL_GPIO_WritePin(mosi->port, mosi->pin, (tx[i] >> bit) & 1U ? GPIO_PIN_SET : GPIO_PIN_RESET);
      DiagDelay();
      HAL_GPIO_WritePin(sck->port, sck->pin, GPIO_PIN_SET);
      DiagDelay();
      in = (uint8_t)(in << 1 | (HAL_GPIO_ReadPin(miso->port, miso->pin) == GPIO_PIN_SET));
      HAL_GPIO_WritePin(sck->port, sck->pin, GPIO_PIN_RESET);
    }
    rx[i] = in;
  }
  HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_SET);
  DiagDelay();
}

/* JEDEC read (command 0x9F, then 3 answer bytes). */
static void DiagJedec(const DiagPin *cs, const DiagPin *mosi, const DiagPin *miso,
                      const DiagPin *sck, uint8_t id[3])
{
  uint8_t tx[4] = {FLASH_CMD_JEDEC, 0, 0, 0};
  uint8_t rx[4];
  DiagPinMode(mosi, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
  DiagPinMode(miso, GPIO_MODE_INPUT, GPIO_PULLUP);
  DiagTransfer(cs, mosi, miso, sck, tx, rx, 4);
  memcpy(id, &rx[1], 3);
}

static void FlashWiringCheck(void)
{
  const DiagPin sck  = {GPIOA, GPIO_PIN_5, "D13"};
  const DiagPin line_d11 = {GPIOA, GPIO_PIN_7, "D11"};
  const DiagPin line_d12 = {GPIOA, GPIO_PIN_6, "D12"};
  /* D8 (PA9) has nothing on it: reading "through" it shows what the line looks like when nobody answers. */
  const DiagPin cs_list[] = {{GPIOB, GPIO_PIN_6, "D10"}, {GPIOC, GPIO_PIN_7, "D9"}, {GPIOA, GPIO_PIN_9, "D8"}};
  const int n_cs = (int)(sizeof(cs_list) / sizeof(cs_list[0]));

  printf("--- Flash wiring check (expected JEDEC C8 40 18) ---\r\n");
  HAL_SPI_DeInit(&hspi1);
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  for (int c = 0; c < n_cs; c++)
  {
    HAL_GPIO_WritePin(cs_list[c].port, cs_list[c].pin, GPIO_PIN_SET);
    DiagPinMode(&cs_list[c], GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
  }
  HAL_GPIO_WritePin(sck.port, sck.pin, GPIO_PIN_RESET);
  DiagPinMode(&sck, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);

  uint8_t idle[2][3];
  for (int swapped = 0; swapped <= 1; swapped++)
  {
    DiagJedec(&cs_list[n_cs - 1], swapped ? &line_d12 : &line_d11, swapped ? &line_d11 : &line_d12,
              &sck, idle[swapped]);
  }

  const char *found_cs = NULL;
  int found_swapped = 0;
  int any_driven = 0;
  for (int c = 0; c < n_cs - 1; c++)
  {
    for (int swapped = 0; swapped <= 1; swapped++)
    {
      const DiagPin *mosi = swapped ? &line_d12 : &line_d11;
      const DiagPin *miso = swapped ? &line_d11 : &line_d12;
      uint8_t id[3];
      DiagJedec(&cs_list[c], mosi, miso, &sck, id);
      int driven = memcmp(id, idle[swapped], 3) != 0;
      any_driven |= driven;
      int ok = id[0] == 0xC8U && id[1] == 0x40U && id[2] == 0x18U;
      printf("  CS=%-3s DI->%s DO->%s : %02X %02X %02X  %s%s\r\n", cs_list[c].name,
             mosi->name, miso->name, id[0], id[1], id[2],
             driven ? "someone answers" : "no answer (same as nothing selected)", ok ? "  <== FLASH" : "");
      if (ok && found_cs == NULL)
      {
        found_cs = cs_list[c].name;
        found_swapped = swapped;
      }
    }
  }

  if (found_cs != NULL && strcmp(found_cs, "D10") == 0 && !found_swapped)
  {
    printf("VERDICT: flash wiring is correct.\r\n");
  }
  else if (found_cs != NULL)
  {
    printf("VERDICT: flash answers with CS on %s%s.\r\n", found_cs,
           found_swapped ? " and DI/DO SWAPPED (DI must go to D11 row, DO to D12 row)" : "");
    if (strcmp(found_cs, "D10") != 0)
    {
      printf("         -> move the flash CS wire to D10.\r\n");
    }
  }
  else if (!any_driven)
  {
    printf("VERDICT: nobody drives the answer line. Check flash DO -> D12 row, CLK -> D13 row,\r\n"
           "         3V3/GND of the flash, and HOLD/WP if the module has them.\r\n");
  }
  else
  {
    printf("VERDICT: something answers but never the flash ID. Likely CLK (D13 row) or DI\r\n"
           "         (D11 row) of the flash is not connected, so it never gets the command.\r\n");
  }

  /* Give the pins back to SPI1 and restore the chip selects. */
  DiagPinMode(&cs_list[2], GPIO_MODE_INPUT, GPIO_NOPULL);
  HAL_SPI_Init(&hspi1);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  printf("--- end of wiring check ---\r\n");
}

/* ---------------- DS18B20 outside thermometer (1-Wire on PA10 = D2) ----------------
   One data wire, open-drain: writing 0 pulls the line low, writing 1 releases it and
   the 4.7k resistor pulls it back up. The sensor answers by pulling low too.
   Every bit is a short, precisely timed low pulse, so we need a microsecond delay. */

static void DelayUsInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* enable the cycle counter (not reset: */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              /* the frame loop measures time with it) */
}

static void DelayUs(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while (DWT->CYCCNT - start < ticks)
  {
  }
}

static void OwLow(void)
{
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
}

static void OwRelease(void)
{
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
}

static int OwRead(void)
{
  return HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin) == GPIO_PIN_SET;
}

/* Reset: low for 480 us, then a present sensor pulls the line low for 60-240 us. */
static int OwReset(void)
{
  OwLow();
  DelayUs(480);
  __disable_irq();
  OwRelease();
  DelayUs(70);
  int present = !OwRead();
  __enable_irq();
  DelayUs(410);
  return present;
}

/* Bits go least significant first. A "1" is a short low pulse, a "0" a long one. */
static void OwWriteByte(uint8_t b)
{
  for (int i = 0; i < 8; i++)
  {
    __disable_irq();
    OwLow();
    if (b & 1U)
    {
      DelayUs(6);
      OwRelease();
      DelayUs(64);
    }
    else
    {
      DelayUs(60);
      OwRelease();
      DelayUs(10);
    }
    __enable_irq();
    b >>= 1;
  }
}

/* Read: short low pulse, release, then sample: the sensor holds the line low for a "0". */
static uint8_t OwReadByte(void)
{
  uint8_t b = 0;
  for (int i = 0; i < 8; i++)
  {
    __disable_irq();
    OwLow();
    DelayUs(3);
    OwRelease();
    DelayUs(10);
    if (OwRead())
    {
      b |= (uint8_t)(1U << i);
    }
    __enable_irq();
    DelayUs(53);
  }
  return b;
}

/* Dallas/Maxim CRC-8 (polynomial x^8 + x^5 + x^4 + 1), checks ROM and scratchpad. */
static uint8_t OwCrc8(const uint8_t *data, int len)
{
  uint8_t crc = 0;
  for (int i = 0; i < len; i++)
  {
    uint8_t in = data[i];
    for (int bit = 0; bit < 8; bit++)
    {
      uint8_t mix = (uint8_t)((crc ^ in) & 1U);
      crc >>= 1;
      if (mix)
      {
        crc ^= 0x8CU;
      }
      in >>= 1;
    }
  }
  return crc;
}

static int Ds18b20Init(void)
{
  DelayUsInit();
  if (!OwReset())
  {
    INIT_PRINTF("DS18B20: no presence pulse (check yellow wire -> D2, 4.7k to 3V3, power)\r\n");
    return -1;
  }
  uint8_t rom[8];
  OwWriteByte(OW_CMD_READ_ROM);
  for (int i = 0; i < 8; i++)
  {
    rom[i] = OwReadByte();
  }
  INIT_PRINTF("DS18B20 ROM: %02X %02X%02X%02X%02X%02X%02X %02X  family 0x%02X (%s), CRC %s\r\n",
         rom[0], rom[6], rom[5], rom[4], rom[3], rom[2], rom[1], rom[7], rom[0],
         rom[0] == 0x28U ? "DS18B20" : "unexpected", OwCrc8(rom, 7) == rom[7] ? "OK" : "BAD");
  return (rom[0] == 0x28U && OwCrc8(rom, 7) == rom[7]) ? 0 : -1;
}

/* Start a temperature conversion (12-bit takes up to 750 ms). */
static void Ds18b20StartConversion(void)
{
  if (OwReset())
  {
    OwWriteByte(OW_CMD_SKIP_ROM);
    OwWriteByte(OW_CMD_CONVERT_T);
  }
}

/* Read the result of the last conversion. Returns 0 and the temperature, or -1. */
static int Ds18b20ReadTemp(double *temp_c)
{
  if (!OwReset())
  {
    return -1;
  }
  OwWriteByte(OW_CMD_SKIP_ROM);
  OwWriteByte(OW_CMD_READ_SCRATCH);
  uint8_t sp[9];
  for (int i = 0; i < 9; i++)
  {
    sp[i] = OwReadByte();
  }
  if (OwCrc8(sp, 8) != sp[8])
  {
    return -1;
  }
  /* A floating or shorted data line reads all zeros, and the CRC of all zeros is 0, so the
     CRC alone passes it (seen as a fake 0.0 C when the probe wire was pulled). Check the
     bits the DS18B20 always sets: config register 0bx11111 and reserved byte 5 = 0xFF. */
  if ((sp[4] & 0x9FU) != 0x1FU || sp[5] != 0xFFU)
  {
    return -1;
  }
  int16_t raw = (int16_t)((uint16_t)sp[1] << 8 | sp[0]);  /* 1/16 degree steps */
  *temp_c = raw / 16.0;
  return 0;
}

/* ---------------- Watchdog and reset cause ----------------
   The IWDG counts down on its own oscillator. If the program does not "feed" it within
   2 s (because it hung), the chip resets. Once started it cannot be stopped. */

static void WatchdogStart(void)
{
  IWDG->KR = IWDG_KEY_START;
  IWDG->KR = IWDG_KEY_ACCESS;       /* unlock the prescaler and reload registers */
  IWDG->PR = IWDG_PRESCALER_64;
  IWDG->RLR = IWDG_RELOAD_2S;
  while (IWDG->SR != 0U)            /* wait until the new values are taken */
  {
  }
  IWDG->KR = IWDG_KEY_FEED;
}

static void WatchdogFeed(void)
{
  IWDG->KR = IWDG_KEY_FEED;
}

/* RCC keeps flags telling why the chip restarted; read them once, then clear them. */
static void ResetCauseCapture(void)
{
  uint32_t csr = RCC->CSR;
  if (csr & RCC_CSR_IWDGRSTF)
  {
    reset_cause = RESET_WATCHDOG;
  }
  else if (csr & RCC_CSR_SFTRSTF)
  {
    reset_cause = RESET_SOFTWARE;
  }
  else if (csr & RCC_CSR_BORRSTF)
  {
    reset_cause = RESET_POWER;      /* power-on also sets the brown-out flag */
  }
  else if (csr & RCC_CSR_PINRSTF)
  {
    reset_cause = RESET_PIN;
  }
  else
  {
    reset_cause = RESET_BROWNOUT;
  }
  RCC->CSR |= RCC_CSR_RMVF;
}

static const char *ResetCauseName(uint8_t c)
{
  switch (c)
  {
    case RESET_POWER: return "power-on";
    case RESET_PIN: return "reset pin / programmer";
    case RESET_SOFTWARE: return "software";
    case RESET_WATCHDOG: return "WATCHDOG (the program hung)";
    default: return "other";
  }
}

/* ---------------- Flight log in the SPI flash ----------------
   Records are appended one after another. The flash only turns bits 1 -> 0, so each
   sector is erased (all 0xFF) before we write into it; the next sector is erased in
   advance, while the current one fills up, so the frame loop never waits for an erase.
   After a reboot the log is scanned to find where it ended and continues from there. */

static uint16_t Crc16Ccitt(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFU;
  while (len--)
  {
    crc ^= (uint16_t)(*data++ << 8);
    for (int bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static uint32_t LogSlotAddr(uint32_t slot)
{
  return LOG_START_ADDR + slot * LOG_RECORD_SIZE;
}

static int FlashBusy(void)
{
  uint8_t cmd = FLASH_CMD_RDSR;
  uint8_t sr = 0;
  FlashSelect();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
  HAL_SPI_Receive(&hspi1, &sr, 1, 100);
  FlashDeselect();
  return (sr & 0x01U) != 0;
}

/* Start a sector erase and return at once; the flash stays busy for up to ~0.4 s. */
static void FlashStartErase(uint32_t addr)
{
  FlashWriteEnable();
  FlashSelect();
  FlashSendCmdAddr(FLASH_CMD_SE, addr);
  FlashDeselect();
}

/* Start a page program and return at once; the flash stays busy for ~1-3 ms. */
static void FlashStartProgram(uint32_t addr, const uint8_t *data, uint16_t len)
{
  FlashWriteEnable();
  FlashSelect();
  FlashSendCmdAddr(FLASH_CMD_PP, addr);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 100);
  FlashDeselect();
}

/* 1 = valid record for this slot, 0 = erased (all 0xFF), -1 = something else (broken). */
static int LogCheckSlot(uint32_t slot, LogRecord *r)
{
  FlashRead(LogSlotAddr(slot), (uint8_t *)r, sizeof(*r));
  if (r->magic == LOG_MAGIC && r->index == slot &&
      r->crc == Crc16Ccitt((const uint8_t *)r, sizeof(*r) - 2))
  {
    return 1;
  }
  const uint8_t *b = (const uint8_t *)r;
  for (uint32_t i = 0; i < sizeof(*r); i++)
  {
    if (b[i] != 0xFFU)
    {
      return -1;
    }
  }
  return 0;
}

/* Find where the log ended: first by sectors (first slot of each sector), then inside
   the last used sector. A record broken by a power cut is skipped, never overwritten. */
static void LogInit(void)
{
  LogRecord r;
  uint32_t sectors = LOG_SLOTS / LOG_PER_SECTOR;
  uint32_t used = 0;
  while (used < sectors && LogCheckSlot(used * LOG_PER_SECTOR, &r) == 1)
  {
    used++;
  }

  if (used == 0)
  {
    log_next = 0;
    log_erased_to = 0;  /* nothing known to be erased: erase before the first write */
  }
  else
  {
    uint32_t base = (used - 1) * LOG_PER_SECTOR;
    uint32_t slot = base;
    uint16_t last_boot = 0;
    while (slot < base + LOG_PER_SECTOR && LogCheckSlot(slot, &r) == 1)
    {
      last_boot = r.boot;
      log_last = r;
      log_last_ok = 1;
      slot++;
    }
    uint32_t skipped = 0;
    while (slot < base + LOG_PER_SECTOR && LogCheckSlot(slot, &r) != 0)
    {
      slot++;  /* half-written record from a power cut: leave it, go past it */
      skipped++;
    }
    log_next = slot;
    log_erased_to = base + LOG_PER_SECTOR;  /* this sector was erased before it was used */
    printf("LOG: found %lu records, last from boot %u, %lu broken slot(s) skipped\r\n",
           (unsigned long)log_next - skipped, last_boot, (unsigned long)skipped);
  }
  printf("LOG: writing from record %lu, room for %lu more (%lu min at 5 Hz)\r\n",
         (unsigned long)log_next, (unsigned long)(LOG_SLOTS - log_next),
         (unsigned long)((LOG_SLOTS - log_next) / 5U / 60U));
}

/* Called once per frame: wrap the frame into a record and queue it. */
static void LogPush(const TelemetryFrame *f)
{
  if (log_full)
  {
    return;
  }
  uint32_t next = (log_q_head + 1U) % LOG_QUEUE_LEN;
  if (next == log_q_tail)
  {
    log_dropped++;  /* flash busy for too long: queue is full */
    return;
  }
  LogRecord *r = &log_queue[log_q_head];
  memset(r, 0xFF, sizeof(*r));
  r->magic = LOG_MAGIC;
  r->boot = boot_id;
  r->reset_cause = reset_cause;
  r->frame = *f;
  r->ground_pa = (float)ground_pa;
  r->asl_offset = (float)asl_offset;
  r->max_alt = fsm.max_alt;
  log_q_head = next;
}

/* Called as often as possible: does at most one flash operation and never waits. */
static void LogService(void)
{
  if (log_full || FlashBusy())
  {
    return;
  }
  if (log_q_tail != log_q_head)
  {
    if (log_next >= LOG_SLOTS)
    {
      log_full = 1;
      printf("LOG: FULL, logging stopped\r\n");
      return;
    }
    if (log_next >= log_erased_to)
    {
      FlashStartErase(LogSlotAddr(log_erased_to));  /* sector not erased yet: do it now */
      log_erased_to += LOG_PER_SECTOR;
      return;
    }
    LogRecord *r = &log_queue[log_q_tail];
    r->index = log_next;
    r->crc = Crc16Ccitt((const uint8_t *)r, sizeof(*r) - 2);
    FlashStartProgram(LogSlotAddr(log_next), (const uint8_t *)r, sizeof(*r));
    log_next++;
    log_written++;
    log_q_tail = (log_q_tail + 1U) % LOG_QUEUE_LEN;
  }
  else if (log_erased_to < log_next + 2U * LOG_PER_SECTOR && log_erased_to < LOG_SLOTS)
  {
    FlashStartErase(LogSlotAddr(log_erased_to));  /* nothing to write: erase ahead */
    log_erased_to += LOG_PER_SECTOR;
  }
}

/* Console command 'D': print the whole log as CSV. */
static void LogDump(void)
{
  uint32_t t0 = HAL_GetTick();
  while (FlashBusy() && HAL_GetTick() - t0 < 1000U)
  {
  }
  printf("LOGDUMP BEGIN %lu\r\n", (unsigned long)log_next);
  printf("index,boot,seq,t_ms,valid,baro_pa,baro_temp_c,baro_rel_m,ax_g,ay_g,az_g,"
         "gx_dps,gy_dps,gz_dps,out_temp_c,gps_fix,gps_sats,gps_lat,gps_lon,gps_alt_m,reset_cause,"
         "state,alt_m,vz_mps,asl_m,sim\r\n");
  uint32_t bad = 0;
  for (uint32_t slot = 0; slot < log_next; slot++)
  {
    WatchdogFeed();
    LogRecord r;
    if (LogCheckSlot(slot, &r) != 1)
    {
      bad++;
      continue;
    }
    const TelemetryFrame *f = &r.frame;
    printf("%lu,%u,%lu,%lu,%u,", (unsigned long)r.index, r.boot, (unsigned long)f->seq,
           (unsigned long)f->t_ms, (unsigned int)f->valid);
    PrintFixed(f->baro_pa, 2);
    printf(",");
    PrintFixed(f->baro_temp_c, 2);
    printf(",");
    PrintFixed(f->baro_rel_m, 2);
    for (int i = 0; i < 3; i++)
    {
      printf(",");
      PrintFixed(f->acc_g[i], 3);
    }
    for (int i = 0; i < 3; i++)
    {
      printf(",");
      PrintFixed(f->gyro_dps[i], 2);
    }
    printf(",");
    PrintFixed(f->out_temp_c, 2);
    printf(",%u,%u,", f->gps_fix, f->gps_sats);
    PrintFixed(f->gps_lat_e7 / 1e7, 7);
    printf(",");
    PrintFixed(f->gps_lon_e7 / 1e7, 7);
    printf(",");
    PrintFixed(f->gps_alt_m, 1);
    printf(",%u,%s,", r.reset_cause, f->state <= FS_LANDED ? FS_NAME[f->state] : "?");
    PrintFixed(f->alt_m, 1);
    printf(",");
    PrintFixed(f->vz_mps, 2);
    printf(",");
    PrintFixed(f->asl_m, 1);
    printf(",%u\r\n", (f->valid & FRAME_SIM) ? 1U : 0U);
  }
  printf("LOGDUMP END records=%lu broken=%lu\r\n", (unsigned long)(log_next - bad), (unsigned long)bad);
}

/* Console command 'E' then 'Y': erase the whole chip for a fresh log (30-100 s). */
static void LogErase(void)
{
  printf("LOG: erase the whole flash? send Y within 5 s\r\n");
  uint8_t c = 0;
  for (int i = 0; i < 50 && c == 0; i++)
  {
    WatchdogFeed();
    if (HAL_UART_Receive(&hlpuart1, &c, 1, 100) != HAL_OK)
    {
      c = 0;
    }
  }
  if (c != 'Y' && c != 'y')
  {
    printf("LOG: erase cancelled\r\n");
    return;
  }
  uint32_t t0 = HAL_GetTick();
  while (FlashBusy() && HAL_GetTick() - t0 < 1000U)
  {
  }
  printf("LOG: erasing chip");
  FlashWriteEnable();
  uint8_t cmd = FLASH_CMD_CHIP_ERASE;
  FlashSelect();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
  FlashDeselect();
  uint32_t start = HAL_GetTick();
  while (FlashBusy() && HAL_GetTick() - start < 200000U)
  {
    for (int i = 0; i < 10; i++)
    {
      WatchdogFeed();
      HAL_Delay(100);
    }
    printf(".");
  }
  printf(" done in %lu s\r\n", (unsigned long)((HAL_GetTick() - start) / 1000U));
  log_next = 0;
  log_erased_to = LOG_SLOTS;  /* whole chip is erased now */
  log_q_head = log_q_tail = 0;
  log_full = 0;
}

/* ---------------- GPS MAX-M10S (USART1, 9600 baud) ---------------- */

/* Interrupt: one byte arrived -> store it and ask for the next one. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    uint16_t next = (uint16_t)((gps_rx_head + 1U) % GPS_RX_BUF_SIZE);
    if (next != gps_rx_tail)
    {
      gps_rx_buf[gps_rx_head] = gps_rx_byte;
      gps_rx_head = next;
    }
    else
    {
      gps_rx_overflow++;  /* main loop is too slow: byte lost */
    }
    gps_rx_total++;
    HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);
  }
}

/* Noise or overrun stops HAL reception; restart it. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);
  }
}

static int HexDigit(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

/* Split "a,b,,c" in place into fields; returns the number of fields. */
static int NmeaSplit(char *s, char *field[], int max)
{
  int n = 0;
  field[n++] = s;
  for (; *s != '\0' && n < max; s++)
  {
    if (*s == ',')
    {
      *s = '\0';
      field[n++] = s + 1;
    }
  }
  return n;
}

/* NMEA "ddmm.mmmm" + hemisphere -> signed decimal degrees. */
static double NmeaToDegrees(const char *value, const char *hemi)
{
  double raw = atof(value);
  int deg = (int)(raw / 100.0);
  double d = deg + (raw - deg * 100.0) / 60.0;
  return (hemi[0] == 'S' || hemi[0] == 'W') ? -d : d;
}

/* One full sentence like "$GNGGA,...*5C" without the line ending. */
static void NmeaHandle(char *line)
{
  /* Checksum = XOR of all chars between '$' and '*', written as 2 hex digits. */
  char *star = strchr(line, '*');
  if (line[0] != '$' || star == NULL || HexDigit(star[1]) < 0 || HexDigit(star[2]) < 0)
  {
    nmea_bad++;
    return;
  }
  uint8_t sum = 0;
  for (char *p = line + 1; p < star; p++)
  {
    sum ^= (uint8_t)*p;
  }
  if (sum != (uint8_t)(HexDigit(star[1]) << 4 | HexDigit(star[2])))
  {
    nmea_bad++;
    return;
  }
  nmea_ok++;
#if GPS_ECHO_NMEA
  printf("%s\r\n", line);
#endif
  *star = '\0';

  char *f[24];
  int n = NmeaSplit(line + 1, f, 24);  /* f[0] = "GNGGA": 2-letter talker + type */
  if (strlen(f[0]) != 5)
  {
    return;
  }
  const char *type = f[0] + 2;

  if (strcmp(type, "GGA") == 0 && n >= 10)
  {
    /* 1 UTC time, 2 lat, 3 N/S, 4 lon, 5 E/W, 6 fix quality, 7 sats used, 8 HDOP, 9 altitude */
    gps_last_gga_ms = HAL_GetTick();
    strncpy(gps.utc, f[1], 6);
    gps.utc[6] = '\0';
    gps.fix_quality = atoi(f[6]);
    gps.sats_used = atoi(f[7]);
    if (gps.fix_quality > 0)
    {
      gps.lat_deg = NmeaToDegrees(f[2], f[3]);
      gps.lon_deg = NmeaToDegrees(f[4], f[5]);
      gps.alt_m = atof(f[9]);
    }
  }
  else if (strcmp(type, "GSV") == 0 && n >= 4)
  {
    /* 1 number of messages, 2 this message number, 3 satellites in view */
    const char *t = strchr(GPS_TALKERS, f[0][1]);
    if (t != NULL && atoi(f[2]) == 1)
    {
      gsv_in_view[t - GPS_TALKERS] = atoi(f[3]);
    }
  }
}

/* Feed one received byte: text goes to NMEA, 0xB5 0x62 ... frames go to UBX. */
static void GpsProcessByte(uint8_t b)
{
  static char line[GPS_LINE_MAX];
  static uint16_t line_len;
  static UbxFrame f;
  static uint8_t state, ck_a, ck_b;
  static uint16_t idx;

  switch (state)
  {
    case 0:  /* outside UBX: collect NMEA text */
      if (b == 0xB5U)
      {
        state = 1;
      }
      else if (b == '$')
      {
        line[0] = '$';
        line_len = 1;
      }
      else if (b == '\n' || b == '\r')
      {
        if (line_len > 0)
        {
          line[line_len] = '\0';
          NmeaHandle(line);
          line_len = 0;
        }
      }
      else if (line_len > 0 && line_len < GPS_LINE_MAX - 1U)
      {
        line[line_len++] = (char)b;
      }
      break;
    case 1: state = (b == 0x62U) ? 2 : 0; break;              /* second sync byte */
    case 2: f.cls = b; ck_a = b; ck_b = ck_a; state = 3; break;
    case 3: f.id = b; ck_a += b; ck_b += ck_a; state = 4; break;
    case 4: f.len = b; ck_a += b; ck_b += ck_a; state = 5; break;
    case 5:
      f.len |= (uint16_t)b << 8;
      ck_a += b; ck_b += ck_a;
      idx = 0;
      state = (f.len > sizeof(f.payload)) ? 0 : (f.len ? 6 : 7);
      break;
    case 6:
      f.payload[idx++] = b;
      ck_a += b; ck_b += ck_a;
      if (idx == f.len) state = 7;
      break;
    case 7: state = (b == ck_a) ? 8 : 0; break;               /* checksum A */
    case 8:                                                    /* checksum B */
      if (b == ck_b)
      {
        f.ready = 1;
        ubx = f;
      }
      state = 0;
      break;
  }
}

/* Drain everything the interrupt has collected so far. */
static void GpsPoll(void)
{
  while (gps_rx_tail != gps_rx_head)
  {
    uint8_t b = gps_rx_buf[gps_rx_tail];
    gps_rx_tail = (uint16_t)((gps_rx_tail + 1U) % GPS_RX_BUF_SIZE);
    GpsProcessByte(b);
  }
}

/* UBX frame: B5 62 class id len(2, LSB first) payload ck_a ck_b (Fletcher checksum). */
static void UbxSend(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
  uint8_t hdr[6] = {0xB5U, 0x62U, cls, id, (uint8_t)len, (uint8_t)(len >> 8)};
  uint8_t ck[2] = {0, 0};
  for (int i = 2; i < 6; i++)
  {
    ck[0] += hdr[i];
    ck[1] += ck[0];
  }
  for (uint16_t i = 0; i < len; i++)
  {
    ck[0] += payload[i];
    ck[1] += ck[0];
  }
  ubx.ready = 0;
  HAL_UART_Transmit(&huart1, hdr, sizeof(hdr), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)payload, len, 100);
  HAL_UART_Transmit(&huart1, ck, sizeof(ck), 100);
}

/* Send a request and wait for the answer.
   Returns 1 on data (want_data) or ACK, 0 on NAK, -1 on timeout. */
static int UbxRequest(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, int want_data)
{
  UbxSend(cls, id, payload, len);
  uint32_t start = HAL_GetTick();
  while (HAL_GetTick() - start < 1000U)
  {
    GpsPoll();
    if (!ubx.ready)
    {
      continue;
    }
    ubx.ready = 0;
    if (want_data && ubx.cls == cls && ubx.id == id)
    {
      return 1;
    }
    if (ubx.cls == UBX_CLASS_ACK && ubx.len >= 2 && ubx.payload[0] == cls && ubx.payload[1] == id)
    {
      if (ubx.id == UBX_ID_NAK) return 0;
      if (!want_data) return 1;
    }
  }
  return -1;
}

/* Read the dynamic model from a layer: 0 = RAM (active), 1 = BBR (kept by backup cell).
   Returns the value, -2 if the layer holds no value (NAK), -1 on timeout. */
static int GpsGetDynModel(uint8_t layer)
{
  uint8_t p[8] = {0x00, layer, 0x00, 0x00,
                  (uint8_t)CFG_NAVSPG_DYNMODEL, (uint8_t)(CFG_NAVSPG_DYNMODEL >> 8),
                  (uint8_t)(CFG_NAVSPG_DYNMODEL >> 16), (uint8_t)(CFG_NAVSPG_DYNMODEL >> 24)};
  int r = UbxRequest(UBX_CLASS_CFG, UBX_ID_VALGET, p, sizeof(p), 1);
  if (r == 0) return -2;
  if (r < 0 || ubx.len < 9) return -1;
  return ubx.payload[8];  /* version, layer, position(2), key(4), value */
}

static const char *DynModelName(int m)
{
  switch (m)
  {
    case 0: return "portable";
    case 2: return "stationary";
    case 3: return "pedestrian";
    case 4: return "automotive";
    case 6: return "airborne<1g";
    case 7: return "airborne<2g";
    case 8: return "AIRBORNE<4g";
    case -1: return "no answer";
    case -2: return "not stored";
    default: return "other";
  }
}

/* Modules are sometimes pre-set to another speed: try common baud rates
   and keep the first one where NMEA checksums pass. Returns the baud or 0. */
static uint32_t GpsDetectBaud(void)
{
  static const uint32_t bauds[] = {9600, 38400, 115200, 57600, 19200, 4800};
  for (unsigned i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++)
  {
    HAL_UART_AbortReceive(&huart1);
    huart1.Init.BaudRate = bauds[i];
    HAL_UART_Init(&huart1);
    gps_rx_tail = gps_rx_head;  /* drop bytes received at the old speed */
    gps_rx_total = 0;
    nmea_ok = 0;
    nmea_bad = 0;
    HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);

    uint32_t start = HAL_GetTick();
    while (nmea_ok < 2 && HAL_GetTick() - start < 1500U)
    {
      GpsPoll();
    }
    printf("GPS baud %6lu: bytes=%lu nmea ok=%lu bad=%lu\r\n", (unsigned long)bauds[i],
           (unsigned long)gps_rx_total, (unsigned long)nmea_ok, (unsigned long)nmea_bad);
    if (nmea_ok >= 2)
    {
      return bauds[i];
    }
  }
  return 0;
}

static void GpsInit(void)
{
  /* 1. Is the module talking, and at what speed? */
  uint32_t baud = GpsDetectBaud();
  if (baud == 0)
  {
    printf("GPS: no valid NMEA at any speed. Check GPS TX -> D0, GND, power\r\n");
    return;
  }
  printf("GPS: NMEA OK at %lu baud\r\n", (unsigned long)baud);

  /* 2. What model did the module wake up with? 8 here after a power cycle = setting survived. */
  int ram = GpsGetDynModel(0);
  int bbr = GpsGetDynModel(1);
  gps_boot_dynmodel = ram;
  printf("GPS dynModel at boot: RAM=%d (%s), BBR=%d (%s)\r\n",
         ram, DynModelName(ram), bbr, DynModelName(bbr));

  /* 3. Set Airborne <4g in RAM (now) and BBR (after power-off, if the backup cell holds). */
  uint8_t p[9] = {0x00, UBX_LAYER_RAM | UBX_LAYER_BBR, 0x00, 0x00,
                  (uint8_t)CFG_NAVSPG_DYNMODEL, (uint8_t)(CFG_NAVSPG_DYNMODEL >> 8),
                  (uint8_t)(CFG_NAVSPG_DYNMODEL >> 16), (uint8_t)(CFG_NAVSPG_DYNMODEL >> 24),
                  DYNMODEL_AIRBORNE_4G};
  int r = UbxRequest(UBX_CLASS_CFG, UBX_ID_VALSET, p, sizeof(p), 0);
  printf("GPS set Airborne<4g: %s\r\n", r == 1 ? "ACK" : (r == 0 ? "NAK" : "no answer"));

  /* 4. Read back to be sure. */
  ram = GpsGetDynModel(0);
  printf("GPS dynModel now: RAM=%d (%s)\r\n", ram, DynModelName(ram));
}

/* Copy the latest parsed GPS data into the frame (GPS itself updates once per second). */
static void GpsFillFrame(TelemetryFrame *f)
{
  int in_view = 0;
  for (unsigned i = 0; i < sizeof(gsv_in_view) / sizeof(gsv_in_view[0]); i++)
  {
    in_view += gsv_in_view[i];
  }
  if (gps_last_gga_ms == 0 || HAL_GetTick() - gps_last_gga_ms > GPS_STALE_MS)
  {
    /* GPS went silent: do not pass old coordinates off as new ones. */
    f->gps_fix = 0;
    f->gps_sats = 0;
    f->gps_in_view = 0;
    return;
  }
  f->valid |= FRAME_OK_GPSLINK;
  f->gps_fix = (uint8_t)gps.fix_quality;
  f->gps_sats = (uint8_t)gps.sats_used;
  f->gps_in_view = (uint8_t)in_view;
  if (gps.fix_quality > 0)
  {
    f->valid |= FRAME_OK_GPSFIX;
    f->gps_lat_e7 = (int32_t)lround(gps.lat_deg * 1e7);
    f->gps_lon_e7 = (int32_t)lround(gps.lon_deg * 1e7);
    f->gps_alt_m = (float)gps.alt_m;
  }
}

/* ---------------- The frame: one line per frame on the console ---------------- */

static void FramePrint(const TelemetryFrame *f)
{
  printf("FRM seq=%lu t=%lu | BARO p=", (unsigned long)f->seq, (unsigned long)f->t_ms);
  PrintFixed(f->baro_pa, 2);
  printf(" T=");
  PrintFixed(f->baro_temp_c, 2);
  printf(" rel=");
  PrintFixed(f->baro_rel_m, 2);
  printf(" | IMU a=");
  PrintFixed(f->acc_g[0], 2);
  printf(",");
  PrintFixed(f->acc_g[1], 2);
  printf(",");
  PrintFixed(f->acc_g[2], 2);
  printf(" g=");
  PrintFixed(f->gyro_dps[0], 1);
  printf(",");
  PrintFixed(f->gyro_dps[1], 1);
  printf(",");
  PrintFixed(f->gyro_dps[2], 1);
  printf(" | GPS fix=%u sats=%u view=%u alt=", f->gps_fix, f->gps_sats, f->gps_in_view);
  PrintFixed(f->gps_alt_m, 1);
  printf(" | OUT T=");
  PrintFixed(f->out_temp_c, 2);
  printf(" | work=%lu valid=0x%02X model@boot=%d", (unsigned long)f->work_us,
         (unsigned int)f->valid, gps_boot_dynmodel);
  printf(" | FLT st=%s h=", f->state <= FS_LANDED ? FS_NAME[f->state] : "?");
  PrintFixed(f->alt_m, 1);
  printf(" vz=");
  PrintFixed(f->vz_mps, 1);
  printf(" asl=");
  PrintFixed(f->asl_m, 1);
  printf(" src=%s%s\r\n", f->asl_src == ASL_GPS ? "GPS" : f->asl_src == ASL_QNH ? "QNH" : "STD",
         (f->valid & FRAME_SIM) ? " SIM" : "");
}

/* ---------------- Flight state machine (HW-18) ---------------- */

static void BuzzerInit(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_4;  /* PB4 = D5 on the Nucleo; FET gate of the buzzer on the flight board */
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

static void BuzzerSet(int on)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 'S' simulation: a real flight profile (5 m/s up, burst at 30 km, parachute down)
   played SIM_SPEED times faster, with +-0.5 m of noise. The 20 s around the burst run
   at real speed, so burst detection is tested on a real fall speed. */
static double SimAltitude(uint32_t now_ms)
{
  const double pre_s = 400.0, burst_m = 30000.0, scale_m = 14600.0;
  const double w_st = pre_s + burst_m / 5.0 - 5.0;  /* flight time 5 s before the burst */
  const double w_r = w_st / SIM_SPEED;              /* ... reached at this real time */
  double r = (now_ms - sim_start_ms) / 1000.0;
  double st = r < w_r ? r * SIM_SPEED
            : r < w_r + 20.0 ? w_st + (r - w_r)
            : w_st + 20.0 + (r - w_r - 20.0) * SIM_SPEED;
  double h;
  if (st < pre_s)
  {
    h = 0.0;
  }
  else if (st < pre_s + burst_m / 5.0)
  {
    h = (st - pre_s) * 5.0;
  }
  else
  {
    /* descent speed 5 m/s * exp(h / 14.6 km): solved in closed form */
    double e = exp(-burst_m / scale_m) + 5.0 * (st - pre_s - burst_m / 5.0) / scale_m;
    h = e >= 1.0 ? 0.0 : -scale_m * log(e);
  }
  sim_rng = sim_rng * 1103515245U + 12345U;
  return h + (((sim_rng >> 16) & 0xFFU) / 255.0 - 0.5);
}

/* Altitudes for this frame and one step of the state machine. */
static void FlightUpdate(TelemetryFrame *f, double press_pa, int baro_ok)
{
  double alt = 0.0;
  int alt_ok = 0;
  int gps_ok = (f->valid & FRAME_OK_GPSFIX) != 0;
  if (baro_ok && ground_pa > 0.0)
  {
    /* Before launch the sea-level reference follows the GPS (it has no weather error). */
    if (fsm.state == FS_PRELAUNCH && !sim_active && gps_ok && f->gps_sats >= 6U)
    {
      double target = f->gps_alt_m - IsaAltitude(press_pa);
      if (asl_src != ASL_GPS)
      {
        asl_offset = target;
        asl_src = ASL_GPS;
        printf("ALT: sea level now calibrated by GPS (%u satellites)\r\n", f->gps_sats);
      }
      else
      {
        asl_offset += 0.02 * (target - asl_offset);  /* ~10 s average */
      }
    }
    f->asl_m = (float)(IsaAltitude(press_pa) + asl_offset);
    alt = IsaAltitude(press_pa) - IsaAltitude(ground_pa);
    alt_ok = 1;
  }
  else if (gps_ok && ground_pa > 0.0)
  {
    /* No barometer: GPS altitude minus the launch point's sea-level altitude. */
    f->asl_m = f->gps_alt_m;
    alt = f->gps_alt_m - (IsaAltitude(ground_pa) + asl_offset);
    alt_ok = 1;
  }
  if (sim_active)
  {
    alt = SimAltitude(f->t_ms);
    alt_ok = 1;
    f->valid |= FRAME_SIM;
    f->asl_m = (float)(alt + IsaAltitude(ground_pa > 0.0 ? ground_pa : 101325.0) + asl_offset);
  }

  FlightState before = fsm.state;
  if (FlightSmUpdate(&fsm, f->t_ms, (float)alt, alt_ok))
  {
    printf("STATE: %s -> %s  h=", FS_NAME[before], FS_NAME[fsm.state]);
    PrintFixed(fsm.alt, 1);
    printf(" m  vz=");
    PrintFixed(fsm.vz, 1);
    printf(" m/s  max=");
    PrintFixed(fsm.max_alt, 1);
    printf(" m%s\r\n", sim_active ? "  (SIM)" : "");
  }
  f->alt_m = fsm.alt;
  f->vz_mps = fsm.vz;
  f->state = (uint8_t)fsm.state;
  f->asl_src = asl_src;
}

/* At boot: if the log says we were flying and this was not a normal start, continue the
   flight instead of calling the current altitude "ground". */
static void FlightResume(int bmp_ok)
{
  FlightSmInit(&fsm, HAL_GetTick());
  if (!log_last_ok || (log_last.frame.valid & FRAME_SIM) ||
      log_last.frame.state < FS_ASCENT || log_last.frame.state > FS_LANDED)
  {
    return;
  }
  int unplanned = reset_cause == RESET_WATCHDOG || reset_cause == RESET_SOFTWARE ||
                  reset_cause == RESET_BROWNOUT;
  int in_air = bmp_ok && ground_pa > 0.0 && ground_pa < log_last.ground_pa - 1000.0;  /* ~90 m up */
  if (!unplanned && !in_air)
  {
    return;  /* power-on or reset button on the ground: a new start */
  }
  ground_pa = log_last.ground_pa;
  asl_offset = log_last.asl_offset;
  asl_src = log_last.frame.asl_src;
  FlightSmResume(&fsm, HAL_GetTick(), (FlightState)log_last.frame.state, log_last.max_alt);
  printf("FLIGHT: resumed %s after a %s reset, launch pressure ", FS_NAME[fsm.state],
         ResetCauseName(reset_cause));
  PrintFixed2(ground_pa);
  printf(" Pa, max altitude ");
  PrintFixed(fsm.max_alt, 0);
  printf(" m\r\n");
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
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  /* The programmer (st-flash) sets "halt on reset" in the debug unit. That register
     survives every reset except power-off, so a watchdog reset would stop the core
     instead of restarting the program. Clear it: no programmer is attached in flight. */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_VC_CORERESET_Msk;
  ResetCauseCapture();
  DelayUsInit();  /* cycle counter: microsecond delays and loop timing */
  FlashDeselect();
  ImuCsInit();
  BuzzerInit();
  HAL_Delay(100);
  printf("\r\n=== hab_bringup: BMP390 + SPI flash + GPS ===\r\n");
#if WIRING_DIAG
  FlashWiringCheck();
#endif

  FlashBootTest();
  printf("RESET cause: %s (boot %u)\r\n", ResetCauseName(reset_cause), boot_id);
  LogInit();

  int imu_ok = ImuInit() == 0;
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
    printf(" Pa, ~");
    PrintFixed(IsaAltitude(ground_pa), 0);
    printf(" m above sea level at standard pressure (GPS refines it)\r\n");
  }
  FlightResume(bmp_ok);

  int ds_ok = Ds18b20Init() == 0;
  if (ds_ok)
  {
    Ds18b20StartConversion();
  }

  GpsInit();

  /* Scheduler state. Each deadline is the previous deadline + period, so small
     delays do not add up over time. Timing is measured with the cycle counter. */
  const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  TelemetryFrame frame;
  memset(&frame, 0, sizeof(frame));
  uint32_t next_ms = HAL_GetTick() + FRAME_PERIOD_MS;
  uint32_t prev_start_cyc = DWT->CYCCNT;
  uint32_t stat_n = 0, stat_work_sum = 0, stat_work_max = 0, stat_print_max = 0;
  uint32_t stat_jitter_max = 0, overruns = 0;
  printf("LOOP: started, period %lu ms\r\n", (unsigned long)FRAME_PERIOD_MS);
  init_verbose = 0;
  uint16_t prev_valid = 0xFFFFU;
  WatchdogStart();
  printf("WATCHDOG: armed, 2 s\r\n");
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Between frames: keep draining GPS bytes and feeding the log to the flash. */
    GpsPoll();
    LogService();
    uint8_t cmd;
    if (HAL_UART_Receive(&hlpuart1, &cmd, 1, 0) == HAL_OK)
    {
      if (cmd == 'D' || cmd == 'd')
      {
        LogDump();
        next_ms = HAL_GetTick() + FRAME_PERIOD_MS;
      }
      else if (cmd == 'E' || cmd == 'e')
      {
        LogErase();
        next_ms = HAL_GetTick() + FRAME_PERIOD_MS;
      }
      else if (cmd == 'S' || cmd == 's')
      {
        if (sim_active)
        {
          sim_active = 0;
          FlightSmInit(&fsm, HAL_GetTick());
          BuzzerSet(0);
          printf("SIM: stopped, back to PRELAUNCH\r\n");
        }
        else if (fsm.state != FS_PRELAUNCH)
        {
          printf("SIM: refused, the real flight is in state %s\r\n", FS_NAME[fsm.state]);
        }
        else
        {
          sim_active = 1;
          sim_start_ms = HAL_GetTick();
          FlightSmInit(&fsm, sim_start_ms);
          printf("SIM: fake flight x40 - burst at 30 km after ~2.7 min, landing after ~4 min,"
                 " LANDED 1 min later. Press S again to stop.\r\n");
        }
      }
      else if (cmd == 'Q' || cmd == 'q')
      {
        uint8_t d[4];
        WatchdogFeed();
        int hpa = -1;
        if (HAL_UART_Receive(&hlpuart1, d, 4, 1500) == HAL_OK)
        {
          hpa = 0;
          for (int i = 0; i < 4; i++)
          {
            hpa = (d[i] >= '0' && d[i] <= '9' && hpa >= 0) ? hpa * 10 + (d[i] - '0') : -1;
          }
        }
        if (hpa >= 870 && hpa <= 1085)
        {
          asl_offset = -IsaAltitude(hpa * 100.0);
          asl_src = ASL_QNH;
          printf("ALT: sea-level pressure set to %d hPa (GPS overrides it before launch)\r\n", hpa);
        }
        else
        {
          printf("ALT: send Q and 4 digits of sea-level pressure in hPa, e.g. Q1009\r\n");
        }
        next_ms = HAL_GetTick() + FRAME_PERIOD_MS;
      }
      else if (cmd == 'H' || cmd == 'h')
      {
        printf("TEST: hanging the loop on purpose, the watchdog should reset the board in ~2 s\r\n");
        while (1)
        {
        }
      }
    }
    if ((int32_t)(HAL_GetTick() - next_ms) < 0)
    {
      continue;
    }

    /* ---- a new frame is due ---- */
    WatchdogFeed();  /* the loop is alive: tell the watchdog */
    uint32_t start_cyc = DWT->CYCCNT;
    uint32_t period_us = (start_cyc - prev_start_cyc) / cycles_per_us;
    prev_start_cyc = start_cyc;
    uint32_t jitter_us = period_us > FRAME_PERIOD_MS * 1000U ? period_us - FRAME_PERIOD_MS * 1000U
                                                             : FRAME_PERIOD_MS * 1000U - period_us;
    next_ms += FRAME_PERIOD_MS;
    if ((int32_t)(HAL_GetTick() - next_ms) >= 0)
    {
      overruns++;  /* we are already late for the next frame: skip ahead, do not pile up */
      next_ms = HAL_GetTick() + FRAME_PERIOD_MS;
    }

    frame.seq++;
    frame.t_ms = HAL_GetTick();
    frame.valid &= FRAME_OK_OUT;  /* keep the last DS18B20 value between its 1 s reads */

    double temp_c = 0.0, press_pa = 0.0;
    if (bmp_ok && BmpReadData(&temp_c, &press_pa) == 0)
    {
      frame.valid |= FRAME_OK_BARO;
      frame.baro_pa = (float)press_pa;
      frame.baro_temp_c = (float)temp_c;
      frame.baro_rel_m = (float)PressureToAltitude(press_pa, ground_pa);
    }
    if (imu_ok && ImuReadReg(IMU_REG_WHO_AM_I) == IMU_WHO_AM_I_VAL)
    {
      /* SPI never "fails" on its own, so check the IMU still answers before trusting data. */
      ImuRead(frame.acc_g, frame.gyro_dps);
      frame.valid |= FRAME_OK_IMU;
    }
    else
    {
      imu_ok = 0;
    }
    if (!ds_ok)
    {
      frame.valid &= (uint16_t)~FRAME_OK_OUT;
    }
    if (ds_ok && frame.seq % DS18B20_EVERY_N == 0U)
    {
      /* Conversion started one second ago is ready: read it, then start the next one. */
      double out_c;
      if (Ds18b20ReadTemp(&out_c) == 0)
      {
        frame.out_temp_c = (float)out_c;
        frame.valid |= FRAME_OK_OUT;
      }
      else
      {
        frame.valid &= (uint16_t)~FRAME_OK_OUT;
      }
      Ds18b20StartConversion();
    }
    GpsFillFrame(&frame);
    FlightUpdate(&frame, press_pa, (frame.valid & FRAME_OK_BARO) != 0);
    BuzzerSet(fsm.state == FS_LANDED && frame.seq % BUZZER_EVERY_N == 0U);

    /* A lost sensor is re-tried every 5 s; the rest of the system keeps flying meanwhile. */
    if (frame.seq % RETRY_EVERY_N == 0U)
    {
      if (!(frame.valid & FRAME_OK_BARO))
      {
        HAL_I2C_DeInit(&hi2c1);  /* a stuck I2C bus is freed by re-initialising it */
        MX_I2C1_Init();
        bmp_ok = BmpInit() == 0;
        if (bmp_ok && ground_pa == 0.0)
        {
          double t, p;
          if (BmpReadData(&t, &p) == 0)
          {
            ground_pa = p;
          }
        }
      }
      if (!imu_ok)
      {
        imu_ok = ImuInit() == 0;
      }
      if (!ds_ok)
      {
        ds_ok = Ds18b20Init() == 0;
        if (ds_ok)
        {
          Ds18b20StartConversion();
        }
      }
    }
    frame.work_us = (DWT->CYCCNT - start_cyc) / cycles_per_us;

    /* Report every change of sensor health once. */
    if (frame.valid != prev_valid)
    {
      static const struct { uint16_t bit; const char *name; } sensors[] = {
        {FRAME_OK_BARO, "BARO"}, {FRAME_OK_IMU, "IMU"}, {FRAME_OK_OUT, "OUT-TEMP"},
        {FRAME_OK_GPSLINK, "GPS-LINK"}, {FRAME_OK_GPSFIX, "GPS-FIX"}};
      for (unsigned i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++)
      {
        uint16_t now = frame.valid & sensors[i].bit;
        uint16_t was = prev_valid & sensors[i].bit;
        if (prev_valid != 0xFFFFU && now != was)
        {
          printf("HEALTH: %s %s\r\n", sensors[i].name, now ? "back" : "lost");
        }
      }
      prev_valid = frame.valid;
    }

    uint32_t print_start = DWT->CYCCNT;
    if (fsm.state != FS_LANDED || frame.seq % LANDED_LOG_EVERY == 0U)
    {
      LogPush(&frame);
    }
    FramePrint(&frame);
    uint32_t print_us = (DWT->CYCCNT - print_start) / cycles_per_us;

    /* ---- loop statistics: is the period stable, how long does a frame take? ---- */
    if (frame.seq > 1U)  /* the first period includes start-up, skip it */
    {
      stat_n++;
      stat_work_sum += frame.work_us;
      stat_work_max = frame.work_us > stat_work_max ? frame.work_us : stat_work_max;
      stat_print_max = print_us > stat_print_max ? print_us : stat_print_max;
      stat_jitter_max = jitter_us > stat_jitter_max ? jitter_us : stat_jitter_max;
    }
    if (stat_n >= LOOP_STATS_EVERY)
    {
      printf("LOOP: period %lu ms  frames %lu  jitter max %lu us  work avg %lu us max %lu us"
             "  print max %lu us  overruns %lu  gps lost %lu  log %lu dropped %lu\r\n",
             (unsigned long)FRAME_PERIOD_MS, (unsigned long)stat_n, (unsigned long)stat_jitter_max,
             (unsigned long)(stat_work_sum / stat_n), (unsigned long)stat_work_max,
             (unsigned long)stat_print_max, (unsigned long)overruns, (unsigned long)gps_rx_overflow,
             (unsigned long)log_next, (unsigned long)log_dropped);
      stat_n = stat_work_sum = stat_work_max = stat_print_max = stat_jitter_max = 0;
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

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
