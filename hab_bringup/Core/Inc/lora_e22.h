/* LoRa E22-400M22S (Semtech SX1268) on SPI2: bring-up check, no transmitting.
 *
 * Wiring (same as the flight PCB, hardware/flight_pcb/SPEC.md):
 *   NSS PB12, SCK PB13, MISO PB14, MOSI PB15, BUSY PC6, DIO1 PC8, NRST PC9,
 *   TXEN PB10, RXEN PB11 (antenna switch; both kept low = switch off).
 *
 * LoraCheck() resets the module, reads its status and the LoRa sync word register
 * (0x0740, reset value 0x14 0x24), writes a test value there, reads it back and resets
 * the module again. Every step names the wire it depends on, so a failed check points at
 * the wire to look at. Nothing is sent over the air: the antenna switch stays off.
 */
#ifndef LORA_E22_H
#define LORA_E22_H

#include "main.h"
#include <stdio.h>

#define LORA_NSS_PORT GPIOB
#define LORA_NSS_PIN GPIO_PIN_12
#define LORA_BUSY_PORT GPIOC
#define LORA_BUSY_PIN GPIO_PIN_6
#define LORA_DIO1_PORT GPIOC
#define LORA_DIO1_PIN GPIO_PIN_8
#define LORA_NRST_PORT GPIOC
#define LORA_NRST_PIN GPIO_PIN_9
#define LORA_SW_PORT GPIOB
#define LORA_TXEN_PIN GPIO_PIN_10
#define LORA_RXEN_PIN GPIO_PIN_11

#define SX_GET_STATUS 0xC0
#define SX_READ_REGISTER 0x1D
#define SX_WRITE_REGISTER 0x0D
#define SX_REG_LORA_SYNC 0x0740

static SPI_HandleTypeDef hspi_lora;
static uint8_t lora_inited;

static void LoraInit(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();

  /* outputs: chip select idle high, reset released, antenna switch off */
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_NRST_PORT, LORA_NRST_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_SW_PORT, LORA_TXEN_PIN | LORA_RXEN_PIN, GPIO_PIN_RESET);
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = LORA_NSS_PIN | LORA_TXEN_PIN | LORA_RXEN_PIN;
  HAL_GPIO_Init(GPIOB, &g);
  g.Pin = LORA_NRST_PIN;
  HAL_GPIO_Init(GPIOC, &g);

  /* BUSY with a pull-up: a loose BUSY wire reads "busy forever" instead of a false "ready" */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Pin = LORA_BUSY_PIN;
  HAL_GPIO_Init(GPIOC, &g);
  g.Pull = GPIO_PULLDOWN;
  g.Pin = LORA_DIO1_PIN;
  HAL_GPIO_Init(GPIOC, &g);

  /* SPI2 on PB13/14/15, AF5. MISO pulled up: a loose MISO wire reads 0xFF */
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_MEDIUM;
  g.Alternate = GPIO_AF5_SPI2;
  g.Pin = GPIO_PIN_13 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &g);
  g.Pull = GPIO_PULLUP;
  g.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOB, &g);

  hspi_lora.Instance = SPI2;
  hspi_lora.Init.Mode = SPI_MODE_MASTER;
  hspi_lora.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_lora.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_lora.Init.CLKPolarity = SPI_POLARITY_LOW;   /* SX126x: SPI mode 0 */
  hspi_lora.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi_lora.Init.NSS = SPI_NSS_SOFT;
  hspi_lora.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;  /* 170 MHz / 64 = 2.7 MHz */
  hspi_lora.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_lora.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_lora.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_lora.Init.CRCPolynomial = 7;
  hspi_lora.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi_lora.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  HAL_SPI_Init(&hspi_lora);
  lora_inited = 1;
}

/* Wait until the module drops BUSY; 0 = ready, -1 = still busy after `ms` */
static int LoraWaitReady(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET)
  {
    if (HAL_GetTick() - t0 > ms)
    {
      return -1;
    }
  }
  return 0;
}

static void LoraXfer(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi_lora, (uint8_t *)tx, rx, n, 50);
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

/* Pulse NRST; returns 1 if BUSY went high during the reset (the module is alive and wired) */
static int LoraReset(void)
{
  HAL_GPIO_WritePin(LORA_NRST_PORT, LORA_NRST_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  int busy_seen = HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET;
  HAL_GPIO_WritePin(LORA_NRST_PORT, LORA_NRST_PIN, GPIO_PIN_SET);
  return busy_seen;
}

static void LoraReadReg(uint16_t addr, uint8_t *out, uint8_t len)
{
  uint8_t tx[8] = {SX_READ_REGISTER, (uint8_t)(addr >> 8), (uint8_t)addr, 0, 0, 0, 0, 0};
  uint8_t rx[8] = {0};
  LoraXfer(tx, rx, (uint16_t)(4 + len));  /* command, 2 address bytes, 1 status byte, data */
  for (uint8_t i = 0; i < len; i++)
  {
    out[i] = rx[4 + i];
  }
}

static void LoraWriteReg(uint16_t addr, const uint8_t *data, uint8_t len)
{
  uint8_t tx[8] = {SX_WRITE_REGISTER, (uint8_t)(addr >> 8), (uint8_t)addr};
  uint8_t rx[8];
  for (uint8_t i = 0; i < len; i++)
  {
    tx[3 + i] = data[i];
  }
  LoraXfer(tx, rx, (uint16_t)(3 + len));
}

/* Full bring-up check, printed line by line. Returns 1 when everything passed. */
static int LoraCheck(void)
{
  static const char *MODE[8] = {"?", "?", "STBY_RC", "STBY_XOSC", "FS", "RX", "TX", "?"};
  if (!lora_inited)
  {
    LoraInit();
  }
  printf("LORA: check of E22-400M22S on SPI2 (nothing is transmitted)\r\n");

  int busy_in_reset = LoraReset();
  int ready = LoraWaitReady(20) == 0;
  printf("LORA: reset - BUSY %s during reset, %s after it\r\n", busy_in_reset ? "high" : "LOW",
         ready ? "low (ready)" : "STILL HIGH");
  if (!ready)
  {
    printf("LORA: FAIL - BUSY never drops. Check: BUSY wire (PC6), VCC 3.3 V and GND, NRST (PC9)\r\n");
    return 0;
  }
  if (!busy_in_reset)
  {
    printf("LORA: note - BUSY did not rise in reset. Check NRST (PC9) and BUSY (PC6) wires\r\n");
  }

  uint8_t tx[2] = {SX_GET_STATUS, 0}, rx[2] = {0};
  LoraXfer(tx, rx, 2);
  uint8_t st = rx[1];
  printf("LORA: status 0x%02X - chip mode %s\r\n", st, MODE[(st >> 4) & 7]);
  if (st == 0xFF || st == 0x00)
  {
    printf("LORA: FAIL - the module answers only %s. Check: MISO (PB14), NSS (PB12), SCK (PB13), VCC and GND\r\n",
           st == 0xFF ? "ones (MISO floating?)" : "zeros (no power or MISO shorted?)");
    return 0;
  }

  uint8_t sync[2];
  LoraWaitReady(10);
  LoraReadReg(SX_REG_LORA_SYNC, sync, 2);
  int sync_ok = sync[0] == 0x14 && sync[1] == 0x24;
  printf("LORA: sync word register 0x0740 = %02X %02X (expected 14 24) - %s\r\n", sync[0], sync[1],
         sync_ok ? "ok" : "WRONG");

  uint8_t test[2] = {0x5A, 0xC3}, back[2];
  LoraWaitReady(10);
  LoraWriteReg(SX_REG_LORA_SYNC, test, 2);
  LoraWaitReady(10);
  LoraReadReg(SX_REG_LORA_SYNC, back, 2);
  int rw_ok = back[0] == test[0] && back[1] == test[1];
  printf("LORA: write 5A C3, read back %02X %02X - %s\r\n", back[0], back[1], rw_ok ? "ok" : "WRONG");

  LoraReset();  /* back to the factory register values */
  LoraWaitReady(20);

  if (sync_ok && rw_ok)
  {
    printf("LORA: OK - the module is wired right and answers. DIO1 is %s.\r\n",
           HAL_GPIO_ReadPin(LORA_DIO1_PORT, LORA_DIO1_PIN) == GPIO_PIN_SET ? "high" : "low");
    return 1;
  }
  printf("LORA: FAIL - status reads but data is wrong. Check: MOSI (PB15) and SCK (PB13), "
         "then MISO (PB14)\r\n");
  return 0;
}

#endif /* LORA_E22_H */
