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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static BmpCalib bmp_cal;
static double ground_pa;  /* pressure at power-on, reference for relative altitude */

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
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
  printf("%ld.%s", x / scale, frac);
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

static void GpsPrintStatus(void)
{
  int in_view = 0;
  for (unsigned i = 0; i < sizeof(gsv_in_view) / sizeof(gsv_in_view[0]); i++)
  {
    in_view += gsv_in_view[i];
  }
  printf("GPS: UTC %.2s:%.2s:%.2s  fix=%d  used=%d  in_view=%d",
         gps.utc, gps.utc + 2, gps.utc + 4, gps.fix_quality, gps.sats_used, in_view);
  if (gps.fix_quality > 0)
  {
    printf("  lat=");
    PrintFixed(gps.lat_deg, 6);
    printf("  lon=");
    PrintFixed(gps.lon_deg, 6);
    printf("  alt=");
    PrintFixed(gps.alt_m, 1);
    printf(" m");
  }
  printf("  (nmea ok=%lu bad=%lu lost=%lu)\r\n",
         (unsigned long)nmea_ok, (unsigned long)nmea_bad, (unsigned long)gps_rx_overflow);
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
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  FlashDeselect();
  HAL_Delay(100);
  printf("\r\n=== hab_bringup: BMP390 + SPI flash + GPS ===\r\n");

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

  GpsInit();
  uint32_t last_print = HAL_GetTick();
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* GPS bytes arrive all the time, so read them on every pass; print once per second. */
    GpsPoll();
    if (HAL_GetTick() - last_print < 1000U)
    {
      continue;
    }
    last_print += 1000U;

    GpsPrintStatus();
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
