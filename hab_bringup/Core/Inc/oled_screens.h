/* What the bench OLED shows (HW-142). Every screen is drawn from one plain struct of
 * values, so the same code runs on the laptop (tools/test_oled.c draws the pictures there).
 *
 *   OledDrawSplash   power-on: HAB-1 and the self-check, one line per real check
 *   OledDrawVerdict  READY / NOT READY
 *   OledDrawFlight   altitude, vertical speed, flight state, altitude graph
 *   OledDrawGps      satellites, fix, UTC time, satellites per constellation
 *   OledDrawSystem   radio counters, temperatures, log, uptime
 *   OledDrawTicker   a running line of text
 *
 * Coordinates are never drawn: the screen ends up in photos and video.
 */
#ifndef OLED_SCREENS_H
#define OLED_SCREENS_H

#include <stdio.h>
#include "oled_ssd1306.h"

#define OLED_TICKER_TEXT "RESILIENCE"
#define OLED_TICKER_STEP 4       /* pixels per frame: 20 pixels a second at 5 Hz */

enum { OLED_CHK_MEMORY, OLED_CHK_IMU, OLED_CHK_BARO, OLED_CHK_TEMP, OLED_CHK_GPS, OLED_CHK_RADIO, OLED_CHK_N };
enum { OLED_NOT_YET, OLED_PASS, OLED_FAIL };
static const char *const OLED_CHK_NAME[OLED_CHK_N] = {"MEMORY", "IMU", "BARO", "TEMP", "GPS", "RADIO"};

typedef struct
{
  uint8_t probe_id;
  uint16_t boot;
  uint8_t chk[OLED_CHK_N];        /* OLED_NOT_YET / OLED_PASS / OLED_FAIL */
  char chk_note[OLED_CHK_N][12];  /* a few words next to the mark, e.g. "98% FREE" */

  const char *state;              /* flight state name */
  const char *mode;               /* "SIM", "LIFT" or NULL */
  float alt_m, vz_mps;
  const char *alt_src;            /* "BARO", "MIX", "GPS", "HOLD", "NONE" or NULL */

  uint8_t gps_link, gps_fix, gps_airborne, sats_used, sats_seen;
  uint8_t seen[4];                /* GPS, GLONASS, Galileo, BeiDou */
  char utc[7];                    /* "hhmmss" or "" */

  uint8_t radio_on;
  int8_t tx_dbm;
  uint32_t freq_khz, sent, failed;

  uint8_t board_temp_ok, out_temp_ok, sensors_ok, sensors_all;
  float board_temp_c, out_temp_c, acc_g;
  uint8_t log_free_pct;
  uint32_t log_min_left, uptime_s;
} OledView;

/* ---------------- Power-on ---------------- */

static void OledDrawSplash(const OledView *v)
{
  char s[24];
  OledClear();
  OledText(0, 0, "HAB-1", 2);
  snprintf(s, sizeof(s), "PROBE %02u", v->probe_id);
  OledTextRight(OLED_W - 1, 0, s, 1);
  if (v->boot)  /* known only after the flash has been read */
  {
    snprintf(s, sizeof(s), "BOOT %u", v->boot);
    OledTextRight(OLED_W - 1, 8, s, 1);
  }
  for (int i = 0; i < OLED_CHK_N; i++)
  {
    int y = 17 + i * 8;
    if (v->chk[i] == OLED_NOT_YET)
    {
      /* only the check that is running now is named; the ones after it are still to come */
      if (i == 0 || v->chk[i - 1] != OLED_NOT_YET)
      {
        OledText(0, y, OLED_CHK_NAME[i], 1);
        OledText(48, y, "..", 1);
      }
      continue;
    }
    OledText(0, y, OLED_CHK_NAME[i], 1);
    OledText(48, y, v->chk[i] == OLED_PASS ? OLED_CH_OK : OLED_CH_FAIL, 1);
    OledText(60, y, v->chk_note[i], 1);
  }
}

static int OledChecksPassed(const OledView *v)
{
  int n = 0;
  for (int i = 0; i < OLED_CHK_N; i++)
  {
    n += v->chk[i] == OLED_PASS;
  }
  return n;
}

static void OledDrawVerdict(const OledView *v)
{
  char s[40];
  int passed = OledChecksPassed(v);
  OledClear();
  snprintf(s, sizeof(s), "HAB-1  PROBE %02u", v->probe_id);
  OledTextCentre(0, s, 1);
  if (passed == OLED_CHK_N)
  {
    OledRect(0, 14, OLED_W, 31, 1);
    oled_ink = 0;
    OledTextCentre(19, "READY", 3);
    oled_ink = 1;
    snprintf(s, sizeof(s), "%d OF %d CHECKS PASSED", passed, OLED_CHK_N);
    OledTextCentre(52, s, 1);
  }
  else
  {
    OledTextCentre(16, "NOT READY", 2);
    OledTextCentre(36, "FAILED:", 1);
    s[0] = 0;
    for (int i = 0; i < OLED_CHK_N; i++)
    {
      if (v->chk[i] != OLED_PASS && strlen(s) + strlen(OLED_CHK_NAME[i]) + 2 < 22)
      {
        strcat(s, s[0] ? " " : "");
        strcat(s, OLED_CHK_NAME[i]);
      }
    }
    OledTextCentre(46, s, 1);
  }
}

/* ---------------- Flight ---------------- */

#define OLED_HIST_N OLED_W
static float oled_hist[OLED_HIST_N];  /* altitude, oldest first */
static int oled_hist_n;

static void OledHistPush(float alt_m)
{
  if (oled_hist_n == OLED_HIST_N)
  {
    memmove(&oled_hist[0], &oled_hist[1], sizeof(oled_hist) - sizeof(oled_hist[0]));
    oled_hist_n--;
  }
  oled_hist[oled_hist_n++] = alt_m;
}

/* The newest point is at the right edge; the scale stretches to what is on screen, but never
   below 4 m top to bottom, so barometer noise on a table stays a flat line. */
static void OledDrawGraph(int y_top, int y_bottom)
{
  if (oled_hist_n == 0)
  {
    return;
  }
  float lo = oled_hist[0], hi = oled_hist[0];
  for (int i = 1; i < oled_hist_n; i++)
  {
    lo = oled_hist[i] < lo ? oled_hist[i] : lo;
    hi = oled_hist[i] > hi ? oled_hist[i] : hi;
  }
  if (hi - lo < 4.0f)
  {
    float mid = (hi + lo) / 2.0f;
    lo = mid - 2.0f;
    hi = mid + 2.0f;
  }
  int prev_y = 0;
  for (int i = 0; i < oled_hist_n; i++)
  {
    int x = OLED_W - oled_hist_n + i;
    int y = y_bottom - (int)((oled_hist[i] - lo) / (hi - lo) * (float)(y_bottom - y_top) + 0.5f);
    OledVLine(x, i == 0 ? y : prev_y, y);
    prev_y = y;
  }
}

static void OledDrawFlight(const OledView *v)
{
  char s[24];
  OledClear();
  OledText(0, 0, v->mode ? v->mode : "HAB-1", 1);
  OledBadgeRight(OLED_W - 2, 1, v->state);

  /* big altitude: tenths of a metre below 1 km, whole metres above */
  float a = v->alt_m;
  OledFix(s, a, (a > -1000.0f && a < 1000.0f) ? 1 : 0);
  int x = OledText(0, 11, s, 3);
  if (x < 96)
  {
    OledText(x, 25, "m", 1);
  }
  s[0] = v->vz_mps >= 0.05f ? '+' : (v->vz_mps <= -0.05f ? '-' : ' ');
  OledFix(&s[1], v->vz_mps < 0 ? -v->vz_mps : v->vz_mps, 1);
  OledTextRight(OLED_W - 1, 11, s, 1);
  OledTextRight(OLED_W - 1, 20, "m/s", 1);
  if (v->alt_src)  /* where the altitude comes from: barometer low, GPS high (HW-15) */
  {
    OledTextRight(OLED_W - 1, 28, v->alt_src, 1);
  }

  OledDrawGraph(35, OLED_H - 1);
}

/* ---------------- Satellites ---------------- */

static void OledDrawGps(const OledView *v)
{
  static const char *const NAME[4] = {"GPS", "GLO", "GAL", "BDS"};
  char s[24];
  OledClear();
  OledText(0, 0, "GPS", 1);
  if (v->gps_airborne)
  {
    OledBadgeRight(OLED_W - 2, 1, "AIRBORNE");
  }
  if (!v->gps_link)
  {
    OledTextCentre(24, "NO DATA", 2);
    OledTextCentre(46, "FROM THE GPS MODULE", 1);
    return;
  }
  snprintf(s, sizeof(s), "%u", v->sats_used);
  int x = OledText(0, 11, s, 2) + 4;
  OledText(x, 11, "SATS IN USE", 1);
  snprintf(s, sizeof(s), "OF %u IN VIEW", v->sats_seen);
  OledText(x, 19, s, 1);

  OledText(0, 30, v->gps_fix ? "FIX OK" : "NO FIX YET", 1);
  if (v->utc[0])
  {
    snprintf(s, sizeof(s), "%.2s:%.2s:%.2s UTC", &v->utc[0], &v->utc[2], &v->utc[4]);
    OledTextRight(OLED_W - 1, 30, s, 1);
  }

  /* satellites in view per constellation: two columns of bars, 2 pixels per satellite */
  for (int i = 0; i < 4; i++)
  {
    int bx = (i % 2) * 66, by = 42 + (i / 2) * 12;
    int n = v->seen[i] > 13 ? 13 : v->seen[i];
    OledText(bx, by, NAME[i], 1);
    OledRect(bx + 20, by, n * 2, 7, 1);
    snprintf(s, sizeof(s), "%u", v->seen[i]);
    OledText(bx + 22 + n * 2, by, s, 1);
  }
}

/* ---------------- Radio and health ---------------- */

static void OledDrawSystem(const OledView *v)
{
  char s[32], a[12], b[12];
  OledClear();
  OledText(0, 0, "RADIO", 1);
  snprintf(s, sizeof(s), "%lu.%03lu MHz", (unsigned long)(v->freq_khz / 1000U), (unsigned long)(v->freq_khz % 1000U));
  OledTextRight(OLED_W - 1, 0, s, 1);

  if (v->radio_on)
  {
    OledBadgeRight(OledTextWidth("TX ON", 1), 11, "TX ON");
  }
  else
  {
    OledText(0, 11, "TX OFF", 1);
  }
  snprintf(s, sizeof(s), "%d dBm", v->tx_dbm);
  OledTextRight(OLED_W - 1, 11, s, 1);
  snprintf(s, sizeof(s), "SENT %lu", (unsigned long)v->sent);
  OledText(0, 21, s, 1);
  snprintf(s, sizeof(s), "FAILED %lu", (unsigned long)v->failed);
  OledTextRight(OLED_W - 1, 21, s, 1);

  OledRect(0, 31, OLED_W, 1, 1);

  snprintf(s, sizeof(s), "IN %s C", v->board_temp_ok ? OledFix(a, v->board_temp_c, 1) : "--");
  OledText(0, 35, s, 1);
  snprintf(s, sizeof(s), "OUT %s C", v->out_temp_ok ? OledFix(b, v->out_temp_c, 1) : "--");
  OledTextRight(OLED_W - 1, 35, s, 1);

  snprintf(s, sizeof(s), "LOG %u%% FREE", v->log_free_pct);
  OledText(0, 45, s, 1);
  snprintf(s, sizeof(s), "%luh%02lum", (unsigned long)(v->log_min_left / 60U), (unsigned long)(v->log_min_left % 60U));
  OledTextRight(OLED_W - 1, 45, s, 1);

  snprintf(s, sizeof(s), "UP %02lu:%02lu:%02lu", (unsigned long)(v->uptime_s / 3600U),
           (unsigned long)(v->uptime_s / 60U % 60U), (unsigned long)(v->uptime_s % 60U));
  OledText(0, 55, s, 1);
  snprintf(s, sizeof(s), "OK %u/%u", v->sensors_ok, v->sensors_all);
  OledTextRight(OLED_W - 1, 55, s, 1);
}

/* ---------------- Running line ---------------- */

/* Big text that runs right to left and comes round again; tick counts frames. */
static void OledDrawTicker(const OledView *v, uint32_t tick)
{
  char s[24];
  int gap = 39;  /* text + gap is a whole number of steps: the loop closes without a jump */
  int period = OledTextWidth(OLED_TICKER_TEXT, 3) + gap;
  int shift = (int)((tick * OLED_TICKER_STEP) % (uint32_t)period);
  OledClear();
  snprintf(s, sizeof(s), "HAB-1  PROBE %02u", v->probe_id);
  OledTextCentre(2, s, 1);
  for (int x = OLED_W - shift; x < OLED_W; x += period)
  {
    OledText(x, 24, OLED_TICKER_TEXT, 3);
  }
  for (int x = OLED_W - shift - period; x > -period; x -= period)
  {
    OledText(x, 24, OLED_TICKER_TEXT, 3);
  }
  OledTextCentre(55, v->state, 1);
}

#endif /* OLED_SCREENS_H */
