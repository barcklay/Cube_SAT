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
#include "telemetry_packet.h"

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


/* ---------------------------------------------------------------------------------------
 * Radio link (HW-6 / HW-13). WRITTEN AND COMPILED, NOT YET RUN ON THE AIR: there is one
 * module on the bench and nobody to receive. LoraRadioSetup() does not transmit and can be
 * run at any time; LoraSend() transmits: antenna on, and only with the second module ready.
 *
 * The module is an SX1268 with a 32 MHz TCXO fed from DIO3 (E22-400M22S manual, 4.2) and an
 * antenna switch driven by TXEN / RXEN. Commands are those of the Semtech SX1261/2/8
 * datasheet, chapter 13.
 * ------------------------------------------------------------------------------------- */
#define LORA_FREQ_HZ      434500000UL  /* inside 433.05-434.79 MHz; CONFIRM the Thai (NBTC) rules */
#define LORA_SF           9U           /* spreading factor 9, 125 kHz, coding rate 4/5: */
#define LORA_BW           0x04U        /* a 28-byte packet is on the air for about 0.2 s */
#define LORA_CR           0x01U
#define LORA_PREAMBLE     8U
#define LORA_TX_DBM_BENCH (-9)         /* lowest power the chip has: two modules on one table */
#define LORA_TCXO_1V8     0x02U        /* TCXO supply from DIO3; 1.8 V is the usual value for E22
                                          modules (from memory, not in the Ebyte manual) */

#define SX_IRQ_TX_DONE  0x0001U
#define SX_IRQ_RX_DONE  0x0002U
#define SX_IRQ_CRC_ERR  0x0040U
#define SX_IRQ_TIMEOUT  0x0200U

static void LoraCmd(const uint8_t *tx, uint16_t n)
{
  uint8_t rx[16];
  LoraWaitReady(50);
  LoraXfer(tx, rx, n);
}

static void LoraSwitch(int tx_on, int rx_on)
{
  HAL_GPIO_WritePin(LORA_SW_PORT, LORA_TXEN_PIN, tx_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LORA_SW_PORT, LORA_RXEN_PIN, rx_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint16_t LoraIrq(void)
{
  const uint8_t tx[4] = {0x12, 0, 0, 0};   /* GetIrqStatus */
  uint8_t rx[4] = {0};
  LoraWaitReady(50);
  LoraXfer(tx, rx, 4);
  return (uint16_t)((rx[2] << 8) | rx[3]);
}

static void LoraPacketParams(uint8_t len)
{
  const uint8_t tx[7] = {0x8C, 0x00, LORA_PREAMBLE, 0x00 /* explicit header */, len,
                         0x01 /* CRC on */, 0x00 /* normal IQ */};
  LoraCmd(tx, 7);
}

/* Configure the radio for our link. Nothing is transmitted. Returns the chip's error word
   (GetDeviceErrors): 0 = the TCXO started and every calibration passed. */
static uint16_t LoraRadioSetup(int8_t tx_dbm)
{
  if (!lora_inited)
  {
    LoraInit();
  }
  LoraSwitch(0, 0);
  LoraReset();
  LoraWaitReady(20);
  {
    const uint8_t standby[2] = {0x80, 0x00};                         /* SetStandby(RC) */
    const uint8_t clr_err[3] = {0x07, 0x00, 0x00};                   /* ClearDeviceErrors */
    const uint8_t tcxo[5] = {0x97, LORA_TCXO_1V8, 0x00, 0x01, 0x40}; /* DIO3 = TCXO, 5 ms to start */
    const uint8_t cal[2] = {0x89, 0x7F};                             /* Calibrate everything */
    const uint8_t lora[2] = {0x8A, 0x01};                            /* SetPacketType(LoRa) */
    const uint32_t frf = (uint32_t)(((uint64_t)LORA_FREQ_HZ << 25) / 32000000ULL);
    const uint8_t freq[5] = {0x86, (uint8_t)(frf >> 24), (uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf};
    const uint8_t img[3] = {0x98, 0x6B, 0x6F};                       /* CalibrateImage 430-440 MHz */
    const uint8_t pa[5] = {0x95, 0x04, 0x07, 0x00, 0x01};            /* SetPaConfig: SX1268, up to +22 dBm */
    const uint8_t txp[3] = {0x8E, (uint8_t)tx_dbm, 0x04};            /* SetTxParams: power, 200 us ramp */
    const uint8_t base[3] = {0x8F, 0x00, 0x00};                      /* buffer base addresses */
    const uint8_t mod[5] = {0x8B, LORA_SF, LORA_BW, LORA_CR, 0x00};  /* low data rate optimise off */
    const uint16_t mask = SX_IRQ_TX_DONE | SX_IRQ_RX_DONE | SX_IRQ_CRC_ERR | SX_IRQ_TIMEOUT;
    const uint8_t irq[9] = {0x08, (uint8_t)(mask >> 8), (uint8_t)mask, (uint8_t)(mask >> 8), (uint8_t)mask, 0, 0, 0, 0};
    LoraCmd(standby, 2);
    LoraCmd(clr_err, 3);
    LoraCmd(tcxo, 5);
    LoraCmd(cal, 2);
    LoraWaitReady(100);
    LoraCmd(lora, 2);
    LoraCmd(freq, 5);
    LoraCmd(img, 3);
    LoraWaitReady(100);
    LoraCmd(pa, 5);
    LoraCmd(txp, 3);
    LoraCmd(base, 3);
    LoraCmd(mod, 5);
    LoraPacketParams(TP_SIZE);
    LoraCmd(irq, 9);
  }
  {
    const uint8_t tx[4] = {0x17, 0, 0, 0};   /* GetDeviceErrors */
    uint8_t rx[4] = {0};
    LoraWaitReady(50);
    LoraXfer(tx, rx, 4);
    return (uint16_t)((rx[2] << 8) | rx[3]);
  }
}

/* Transmit one packet and wait for "sent" on DIO1 (at most 1.5 s, under the 2 s watchdog).
   1 = sent, 0 = the radio did not report TxDone. */
static int LoraSend(const uint8_t *data, uint8_t len)
{
  uint8_t tx[3 + 64] = {0x0E, 0x00};          /* WriteBuffer at offset 0 */
  const uint8_t clr[3] = {0x02, 0xFF, 0xFF};  /* ClearIrqStatus */
  const uint8_t go[4] = {0x83, 0x01, 0x77, 0x00};  /* SetTx, timeout 1.5 s (15.625 us units) */
  if (len > 64U)
  {
    return 0;
  }
  for (uint8_t i = 0; i < len; i++)
  {
    tx[2 + i] = data[i];
  }
  LoraPacketParams(len);
  LoraCmd(tx, (uint16_t)(2 + len));
  LoraCmd(clr, 3);
  LoraSwitch(1, 0);
  LoraCmd(go, 4);
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(LORA_DIO1_PORT, LORA_DIO1_PIN) == GPIO_PIN_RESET && HAL_GetTick() - t0 < 1600U)
  {
  }
  uint16_t irq = LoraIrq();
  LoraCmd(clr, 3);
  LoraSwitch(0, 0);
  return (irq & SX_IRQ_TX_DONE) != 0;
}

/* Listen for up to `ms` (keep it under the watchdog period). Returns the packet length,
   0 = nothing came, -1 = a packet came with a bad radio CRC. */
static int LoraListen(uint8_t *out, uint8_t max, uint32_t ms, int *rssi_dbm, int *snr_db)
{
  const uint8_t clr[3] = {0x02, 0xFF, 0xFF};
  const uint8_t go[4] = {0x82, 0xFF, 0xFF, 0xFF};   /* SetRx, continuous */
  const uint8_t standby[2] = {0x80, 0x00};
  LoraCmd(clr, 3);
  LoraSwitch(0, 1);
  LoraCmd(go, 4);
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(LORA_DIO1_PORT, LORA_DIO1_PIN) == GPIO_PIN_RESET && HAL_GetTick() - t0 < ms)
  {
  }
  uint16_t irq = LoraIrq();
  int n = 0;
  if (irq & SX_IRQ_RX_DONE)
  {
    const uint8_t st[4] = {0x13, 0, 0, 0};    /* GetRxBufferStatus: length, start */
    const uint8_t ps[5] = {0x14, 0, 0, 0, 0}; /* GetPacketStatus: RSSI, SNR */
    uint8_t rs[4] = {0}, rp[5] = {0};
    LoraWaitReady(50);
    LoraXfer(st, rs, 4);
    LoraWaitReady(50);
    LoraXfer(ps, rp, 5);
    *rssi_dbm = -(int)rp[2] / 2;
    *snr_db = (int8_t)rp[3] / 4;
    n = rs[2] > max ? max : rs[2];
    uint8_t tx[3 + 64] = {0x1E, rs[3], 0x00}, rx[3 + 64] = {0};  /* ReadBuffer */
    if (n > 64)
    {
      n = 64;
    }
    LoraWaitReady(50);
    LoraXfer(tx, rx, (uint16_t)(3 + n));
    for (int i = 0; i < n; i++)
    {
      out[i] = rx[3 + i];
    }
    if (irq & SX_IRQ_CRC_ERR)
    {
      n = -1;
    }
  }
  LoraCmd(standby, 2);
  LoraCmd(clr, 3);
  LoraSwitch(0, 0);
  return n;
}

/* ---- Continuous receive for the ground station (HW-29) --------------------------------
   LoraListen() above stops the receiver at the end of every call and would cut a packet
   that is arriving at that moment. Here the radio stays in receive all the time:
   LoraRxStart() once, then LoraRxPoll() as often as possible. */
#ifdef HAB_GROUND_STATION
static void LoraRxStart(void)
{
  const uint8_t clr[3] = {0x02, 0xFF, 0xFF};
  const uint8_t go[4] = {0x82, 0xFF, 0xFF, 0xFF};   /* SetRx, continuous: stays in receive after a packet */
  LoraCmd(clr, 3);
  LoraSwitch(0, 1);
  LoraCmd(go, 4);
}

/* Returns the packet length, 0 = nothing new, -1 = a packet came with a bad radio CRC.
   The receiver keeps running. */
static int LoraRxPoll(uint8_t *out, uint8_t max, int *rssi_dbm, int *snr_db)
{
  const uint8_t clr[3] = {0x02, 0xFF, 0xFF};
  if (HAL_GPIO_ReadPin(LORA_DIO1_PORT, LORA_DIO1_PIN) == GPIO_PIN_RESET)
  {
    return 0;
  }
  uint16_t irq = LoraIrq();
  int n = 0;
  if (irq & SX_IRQ_RX_DONE)
  {
    const uint8_t st[4] = {0x13, 0, 0, 0};    /* GetRxBufferStatus: length, start */
    const uint8_t ps[5] = {0x14, 0, 0, 0, 0}; /* GetPacketStatus: RSSI, SNR */
    uint8_t rs[4] = {0}, rp[5] = {0};
    uint8_t tx[3 + 64] = {0x1E, 0x00, 0x00}, rx[3 + 64] = {0};  /* ReadBuffer */
    LoraWaitReady(50);
    LoraXfer(st, rs, 4);
    LoraWaitReady(50);
    LoraXfer(ps, rp, 5);
    *rssi_dbm = -(int)rp[2] / 2;
    *snr_db = (int8_t)rp[3] / 4;
    n = rs[2] > max ? max : rs[2];
    if (n > 64)
    {
      n = 64;
    }
    tx[1] = rs[3];
    LoraWaitReady(50);
    LoraXfer(tx, rx, (uint16_t)(3 + n));
    for (int i = 0; i < n; i++)
    {
      out[i] = rx[3 + i];
    }
    if (irq & SX_IRQ_CRC_ERR)
    {
      n = -1;
    }
  }
  LoraCmd(clr, 3);
  return n;
}

/* Signal level on the channel right now, dBm: with nobody transmitting this is the noise
   floor of the place (GetRssiInst). Only meaningful while receiving. */
static int LoraNoiseDbm(void)
{
  const uint8_t tx[3] = {0x15, 0, 0};
  uint8_t rx[3] = {0};
  LoraWaitReady(50);
  LoraXfer(tx, rx, 3);
  return -(int)rx[2] / 2;
}
#endif /* HAB_GROUND_STATION */

#endif /* LORA_E22_H */
