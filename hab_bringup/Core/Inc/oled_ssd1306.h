/* Small OLED on the bench (HW-142): SSD1306, 128x64, one colour, I2C address 0x3C.
 *
 * It hangs on the same two wires as the barometer (I2C1: SCL D15, SDA D14) and is only a
 * bench / ground thing for photos and video: the flight build does not touch it.
 *
 * The picture is drawn into a buffer in RAM; OledFlush() then sends only the bytes that
 * differ from what the screen already shows, and no more than a given number per call.
 * A whole picture is 1 KB = 0.1 s on a 100 kHz bus, too long for one 5 Hz frame, so the
 * main loop sends it in pieces. A screen that is not plugged in, or is pulled out while
 * running, switches the driver off: the rest of the firmware never waits for it.
 *
 * Font: the classic 5x7 dot-matrix characters in a 6x8 cell, drawn at any whole scale.
 */
#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include <stdint.h>
#include <string.h>

#define OLED_W      128
#define OLED_H      64
#define OLED_PAGES  8            /* the chip stores the picture as 8 rows of 8-pixel-high bytes */
#define OLED_ADDR   (0x3CU << 1)
#define OLED_CH_OK   "\x7f"      /* check mark */
#define OLED_CH_FAIL "\x80"      /* cross */

static uint8_t oled_fb[OLED_W * OLED_PAGES];     /* the picture we want */
static uint8_t oled_shown[OLED_W * OLED_PAGES];  /* the picture the screen has */
static uint8_t oled_ok;                          /* the screen answered and is in use */
static uint8_t oled_ink = 1;                     /* 1 = light pixels on dark, 0 = dark on light */
static uint8_t oled_next_page;
static uint8_t oled_unknown;                     /* bit per row: the screen content is not known */

static const uint8_t OLED_FONT[][5] = {
  {0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5F, 0x00, 0x00}, {0x00, 0x07, 0x00, 0x07, 0x00}, /*   ! " */
  {0x14, 0x7F, 0x14, 0x7F, 0x14}, {0x24, 0x2A, 0x7F, 0x2A, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62}, /* # $ % */
  {0x36, 0x49, 0x55, 0x22, 0x50}, {0x00, 0x05, 0x03, 0x00, 0x00}, {0x00, 0x1C, 0x22, 0x41, 0x00}, /* & ' ( */
  {0x00, 0x41, 0x22, 0x1C, 0x00}, {0x14, 0x08, 0x3E, 0x08, 0x14}, {0x08, 0x08, 0x3E, 0x08, 0x08}, /* ) * + */
  {0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08}, {0x00, 0x60, 0x60, 0x00, 0x00}, /* , - . */
  {0x20, 0x10, 0x08, 0x04, 0x02}, {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00}, /* / 0 1 */
  {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31}, {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 2 3 4 */
  {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03}, /* 5 6 7 */
  {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}, {0x00, 0x36, 0x36, 0x00, 0x00}, /* 8 9 : */
  {0x00, 0x56, 0x36, 0x00, 0x00}, {0x08, 0x14, 0x22, 0x41, 0x00}, {0x14, 0x14, 0x14, 0x14, 0x14}, /* ; < = */
  {0x00, 0x41, 0x22, 0x14, 0x08}, {0x02, 0x01, 0x51, 0x09, 0x06}, {0x32, 0x49, 0x79, 0x41, 0x3E}, /* > ? @ */
  {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22}, /* A B C */
  {0x7F, 0x41, 0x41, 0x22, 0x1C}, {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01}, /* D E F */
  {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F}, {0x00, 0x41, 0x7F, 0x41, 0x00}, /* G H I */
  {0x20, 0x40, 0x41, 0x3F, 0x01}, {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40}, /* J K L */
  {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* M N O */
  {0x7F, 0x09, 0x09, 0x09, 0x06}, {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46}, /* P Q R */
  {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01}, {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* S T U */
  {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63}, /* V W X */
  {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}, {0x00, 0x7F, 0x41, 0x41, 0x00}, /* Y Z [ */
  {0x02, 0x04, 0x08, 0x10, 0x20}, {0x00, 0x41, 0x41, 0x7F, 0x00}, {0x04, 0x02, 0x01, 0x02, 0x04}, /* \ ] ^ */
  {0x40, 0x40, 0x40, 0x40, 0x40}, {0x00, 0x01, 0x02, 0x04, 0x00}, {0x20, 0x54, 0x54, 0x54, 0x78}, /* _ ` a */
  {0x7F, 0x48, 0x44, 0x44, 0x38}, {0x38, 0x44, 0x44, 0x44, 0x20}, {0x38, 0x44, 0x44, 0x48, 0x7F}, /* b c d */
  {0x38, 0x54, 0x54, 0x54, 0x18}, {0x08, 0x7E, 0x09, 0x01, 0x02}, {0x0C, 0x52, 0x52, 0x52, 0x3E}, /* e f g */
  {0x7F, 0x08, 0x04, 0x04, 0x78}, {0x00, 0x44, 0x7D, 0x40, 0x00}, {0x20, 0x40, 0x44, 0x3D, 0x00}, /* h i j */
  {0x7F, 0x10, 0x28, 0x44, 0x00}, {0x00, 0x41, 0x7F, 0x40, 0x00}, {0x7C, 0x04, 0x18, 0x04, 0x78}, /* k l m */
  {0x7C, 0x08, 0x04, 0x04, 0x78}, {0x38, 0x44, 0x44, 0x44, 0x38}, {0x7C, 0x14, 0x14, 0x14, 0x08}, /* n o p */
  {0x08, 0x14, 0x14, 0x18, 0x7C}, {0x7C, 0x08, 0x04, 0x04, 0x08}, {0x48, 0x54, 0x54, 0x54, 0x20}, /* q r s */
  {0x04, 0x3F, 0x44, 0x40, 0x20}, {0x3C, 0x40, 0x40, 0x20, 0x7C}, {0x1C, 0x20, 0x40, 0x20, 0x1C}, /* t u v */
  {0x3C, 0x40, 0x30, 0x40, 0x3C}, {0x44, 0x28, 0x10, 0x28, 0x44}, {0x0C, 0x50, 0x50, 0x50, 0x3C}, /* w x y */
  {0x44, 0x64, 0x54, 0x4C, 0x44}, {0x00, 0x08, 0x36, 0x41, 0x00}, {0x00, 0x00, 0x7F, 0x00, 0x00}, /* z { | */
  {0x00, 0x41, 0x36, 0x08, 0x00}, {0x08, 0x04, 0x08, 0x10, 0x08},                                 /* } ~ */
  {0x18, 0x30, 0x18, 0x0C, 0x06}, {0x22, 0x14, 0x08, 0x14, 0x22},                                 /* check, cross */
};
#define OLED_FONT_N ((int)(sizeof(OLED_FONT) / sizeof(OLED_FONT[0])))

/* ---------------- Drawing into the buffer ---------------- */

static void OledClear(void)
{
  memset(oled_fb, 0, sizeof(oled_fb));
  oled_ink = 1;
}

static void OledPixel(int x, int y, int on)
{
  if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H)
  {
    return;
  }
  if (on)
  {
    oled_fb[(y >> 3) * OLED_W + x] |= (uint8_t)(1U << (y & 7));
  }
  else
  {
    oled_fb[(y >> 3) * OLED_W + x] &= (uint8_t)~(1U << (y & 7));
  }
}

static void OledRect(int x, int y, int w, int h, int on)
{
  for (int j = y; j < y + h; j++)
  {
    for (int i = x; i < x + w; i++)
    {
      OledPixel(i, j, on);
    }
  }
}

/* Vertical line between two heights, either way round: joins the dots of a graph. */
static void OledVLine(int x, int y0, int y1)
{
  if (y0 > y1)
  {
    int t = y0;
    y0 = y1;
    y1 = t;
  }
  OledRect(x, y0, 1, y1 - y0 + 1, 1);
}

static int OledTextWidth(const char *s, int scale)
{
  return (int)strlen(s) * 6 * scale - (*s ? scale : 0);
}

/* Text with its top left corner at x, y. Returns the x right after it. */
static int OledText(int x, int y, const char *s, int scale)
{
  for (; *s; s++)
  {
    int g = (uint8_t)*s - 32;
    if (g < 0 || g >= OLED_FONT_N)
    {
      g = '?' - 32;
    }
    for (int col = 0; col < 5; col++)
    {
      uint8_t bits = OLED_FONT[g][col];
      for (int row = 0; row < 7; row++)
      {
        if (bits & (1U << row))
        {
          OledRect(x + col * scale, y + row * scale, scale, scale, oled_ink);
        }
      }
    }
    x += 6 * scale;
  }
  return x;
}

/* Text that ends at the pixel column x_right. */
static void OledTextRight(int x_right, int y, const char *s, int scale)
{
  OledText(x_right + 1 - OledTextWidth(s, scale), y, s, scale);
}

static void OledTextCentre(int y, const char *s, int scale)
{
  OledText((OLED_W - OledTextWidth(s, scale)) / 2, y, s, scale);
}

/* A label in a filled box: dark letters on a light patch. Ends at x_right. */
static void OledBadgeRight(int x_right, int y, const char *s)
{
  int w = OledTextWidth(s, 1);
  OledRect(x_right - w - 1, y - 1, w + 2, 9, 1);
  oled_ink = 0;
  OledText(x_right - w, y, s, 1);
  oled_ink = 1;
}

/* Number with a fixed count of decimals (0..2): the firmware's printf has no %f. */
static char *OledFix(char *out, double v, int decimals)
{
  long scale = decimals == 2 ? 100 : (decimals == 1 ? 10 : 1);
  long n = (long)((v < 0 ? -v : v) * scale + 0.5);
  char *p = out;
  char digits[12];
  int nd = 0;
  if (v < 0 && n > 0)
  {
    *p++ = '-';
  }
  long whole = n / scale, frac = n % scale;
  do
  {
    digits[nd++] = (char)('0' + whole % 10);
    whole /= 10;
  } while (whole > 0 && nd < 10);
  while (nd > 0)
  {
    *p++ = digits[--nd];
  }
  if (decimals > 0)
  {
    *p++ = '.';
    if (decimals == 2)
    {
      *p++ = (char)('0' + frac / 10);
    }
    *p++ = (char)('0' + frac % 10);
  }
  *p = 0;
  return out;
}

/* ---------------- The wires ---------------- */

#ifdef OLED_HOST_TEST
/* On the laptop the "screen" is a counter: the test checks what would have been sent. */
static uint32_t oled_bus_bytes;
static int oled_bus_fail;
static int OledBusWrite(uint8_t control, const uint8_t *data, uint16_t len)
{
  (void)control;
  (void)data;
  if (oled_bus_fail)
  {
    return -1;
  }
  oled_bus_bytes += len;
  return 0;
}
static int OledBusProbe(void) { return oled_bus_fail ? -1 : 0; }
#else
#include "main.h"
extern I2C_HandleTypeDef hi2c1;
/* control 0x00 = the bytes are commands, 0x40 = they are picture data */
static int OledBusWrite(uint8_t control, const uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, control, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 30) == HAL_OK
             ? 0 : -1;
}
static int OledBusProbe(void)
{
  return HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR, 2, 5) == HAL_OK ? 0 : -1;
}
#endif

/* Looks for the screen and switches it on. Returns 1 when a screen is there. */
static int OledInit(void)
{
  static const uint8_t setup[] = {
    0xAE,        /* display off while it is being set up */
    0xD5, 0x80,  /* clock divider: default */
    0xA8, 0x3F,  /* 64 rows */
    0xD3, 0x00,  /* no vertical shift */
    0x40,        /* picture starts at row 0 */
    0x8D, 0x14,  /* internal charge pump on: the panel needs ~7 V made from 3.3 V */
    0x20, 0x00,  /* horizontal addressing: the write position walks along the row by itself */
    0xA1, 0xC8,  /* orientation: pins at the top */
    0xDA, 0x12,  /* row wiring of the 128x64 panel */
    0x81, 0xCF,  /* brightness */
    0xD9, 0xF1,  /* pre-charge */
    0xDB, 0x40,  /* VCOMH level */
    0xA4, 0xA6,  /* show the memory, not inverted */
    0x2E,        /* no scrolling */
    0xAF,        /* display on */
  };
  oled_ok = 0;
  if (OledBusProbe() != 0 || OledBusWrite(0x00, setup, sizeof(setup)) != 0)
  {
    return 0;
  }
  oled_unknown = 0xFF;  /* whatever the screen shows now, send every row once */
  oled_ok = 1;
  return 1;
}

/* Sends what differs between the buffer and the screen, at most max_bytes of picture data
   (one byte takes about 90 us on the 100 kHz bus). Returns the number of bytes sent. */
static uint32_t OledFlush(uint32_t max_bytes)
{
  uint32_t sent = 0;
  if (!oled_ok)
  {
    return 0;
  }
  int first = oled_next_page;  /* carry on where the last call stopped: no row is starved */
  for (int n = 0; n < OLED_PAGES && sent < max_bytes; n++)
  {
    int page = (first + n) % OLED_PAGES;
    const uint8_t *want = &oled_fb[page * OLED_W];
    uint8_t *have = &oled_shown[page * OLED_W];
    int x0 = 0, x1 = OLED_W - 1;
    if (!(oled_unknown & (1U << page)))
    {
      while (x0 < OLED_W && want[x0] == have[x0])
      {
        x0++;
      }
      if (x0 == OLED_W)
      {
        continue;  /* this row is already right */
      }
      while (want[x1] == have[x1])
      {
        x1--;
      }
    }
    uint8_t where[6] = {0x21, (uint8_t)x0, (uint8_t)x1, 0x22, (uint8_t)page, (uint8_t)page};
    if (OledBusWrite(0x00, where, sizeof(where)) != 0 ||
        OledBusWrite(0x40, &want[x0], (uint16_t)(x1 - x0 + 1)) != 0)
    {
      oled_ok = 0;  /* unplugged or the bus is stuck: stop using it until it is found again */
      return sent;
    }
    memcpy(&have[x0], &want[x0], (size_t)(x1 - x0 + 1));
    oled_unknown &= (uint8_t)~(1U << page);
    sent += (uint32_t)(x1 - x0 + 1);
    oled_next_page = (uint8_t)((page + 1) % OLED_PAGES);
  }
  return sent;
}

#endif /* OLED_SSD1306_H */
