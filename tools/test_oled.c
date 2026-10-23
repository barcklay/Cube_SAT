/* Host test of the bench OLED (HW-142): draws every screen on the laptop with the same
 * code the firmware runs, and checks the part that talks to the screen in small pieces.
 *
 *     cc -O2 -Wall -o /tmp/test_oled tools/test_oled.c && /tmp/test_oled
 *     /tmp/test_oled some/folder     also saves each screen there as a .pbm picture
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define OLED_HOST_TEST
#include "../hab_bringup/Core/Inc/oled_screens.h"

static int fails;
static const char *out_dir;

static void check(int cond, const char *what)
{
  if (!cond)
  {
    fails++;
    printf("FAIL: %s\n", what);
  }
}

static int lit(void)
{
  int n = 0;
  for (int y = 0; y < OLED_H; y++)
  {
    for (int x = 0; x < OLED_W; x++)
    {
      n += (oled_fb[(y >> 3) * OLED_W + x] >> (y & 7)) & 1;
    }
  }
  return n;
}

static void save(const char *name)
{
  char path[512];
  if (!out_dir)
  {
    return;
  }
  snprintf(path, sizeof(path), "%s/%s.pbm", out_dir, name);
  FILE *f = fopen(path, "w");
  if (!f)
  {
    return;
  }
  fprintf(f, "P1\n%d %d\n", OLED_W, OLED_H);
  for (int y = 0; y < OLED_H; y++)
  {
    for (int x = 0; x < OLED_W; x++)
    {
      fputc(((oled_fb[(y >> 3) * OLED_W + x] >> (y & 7)) & 1) ? '1' : '0', f);
    }
    fputc('\n', f);
  }
  fclose(f);
}

int main(int argc, char **argv)
{
  char s[16];
  out_dir = argc > 1 ? argv[1] : NULL;

  check(!strcmp(OledFix(s, 0.04, 1), "0.0"), "0.04 -> 0.0, no minus zero");
  check(!strcmp(OledFix(s, -0.04, 1), "0.0"), "-0.04 -> 0.0, no minus zero");
  check(!strcmp(OledFix(s, -12.34, 1), "-12.3"), "-12.34 -> -12.3");
  check(!strcmp(OledFix(s, 29999.6, 0), "30000"), "29999.6 -> 30000");
  check(!strcmp(OledFix(s, 1.005, 2), "1.01") || !strcmp(s, "1.00"), "two decimals");
  check(!strcmp(OledFix(s, 9.96, 1), "10.0"), "9.96 -> 10.0");
  check(OledTextWidth("HAB-1", 2) == 58 && OledTextWidth("", 1) == 0, "text width");

  OledView v = {0};
  v.probe_id = 1;
  v.boot = 153;
  v.state = "PRELAUNCH";

  /* power-on: the lines come one by one, a line that has not run yet shows no result */
  OledDrawSplash(&v);
  int before = lit();
  save("splash_0");
  static const char *const NOTE[OLED_CHK_N] = {"98% FREE", "", "1002 hPa", "", "AIRBORNE", "434.5 MHz"};
  for (int i = 0; i < OLED_CHK_N; i++)
  {
    v.chk[i] = OLED_PASS;
    strcpy(v.chk_note[i], NOTE[i]);
    OledDrawSplash(&v);
    check(lit() > before, "each finished check adds something to the splash");
    before = lit();
    if (i == 2)
    {
      save("splash_3");
    }
  }
  save("splash_6");
  OledDrawVerdict(&v);
  save("verdict_ready");
  check(lit() > OLED_W * 20, "READY is a filled banner");
  v.chk[OLED_CHK_GPS] = OLED_FAIL;
  v.chk[OLED_CHK_RADIO] = OLED_FAIL;
  OledDrawSplash(&v);
  save("splash_fail");
  OledDrawVerdict(&v);
  save("verdict_not_ready");
  v.chk[OLED_CHK_GPS] = v.chk[OLED_CHK_RADIO] = OLED_PASS;

  /* the wire: first flush sends everything in pieces, an unchanged picture sends nothing */
  OledDrawSplash(&v);
  check(OledFlush(400) == 0, "no screen found yet: nothing is sent");
  check(OledInit() == 1, "screen found");
  uint32_t total = 0, calls = 0, n;
  while ((n = OledFlush(400)) > 0)
  {
    check(n <= 400 + OLED_W, "one call stays near its budget");
    total += n;
    calls++;
  }
  check(total == sizeof(oled_fb) && calls >= 2, "the first picture goes out whole, in several calls");
  check(memcmp(oled_fb, oled_shown, sizeof(oled_fb)) == 0, "screen copy equals the buffer");
  check(OledFlush(400) == 0, "nothing changed: nothing sent");
  OledPixel(100, 20, 1);
  check(OledFlush(400) == 1, "one pixel changed: one byte sent");
  OledText(0, 56, "X", 1);
  oled_bus_fail = 1;
  check(OledFlush(400) == 0 && oled_ok == 0, "screen pulled out: the driver switches itself off");
  check(OledFlush(400) == 0, "and stays quiet");
  oled_bus_fail = 0;
  check(OledInit() == 1 && OledFlush(100000) == sizeof(oled_fb), "plugged back: found and redrawn");

  /* live screens */
  v.alt_m = 0.3f;
  v.vz_mps = 0.0f;
  for (int i = 0; i < 40; i++)
  {
    OledHistPush(0.1f * (float)sin(i));
  }
  OledDrawFlight(&v);
  save("flight_ground");
  for (int i = 0; i < 200; i++)  /* a lift: up 30 m, a pause, down again */
  {
    float t = (float)i;
    OledHistPush(t < 60 ? t * 0.5f : (t < 110 ? 30.0f : (t < 170 ? 30.0f - (t - 110) * 0.5f : 0.0f)));
  }
  check(oled_hist_n == OLED_HIST_N, "the graph keeps the last 128 points");
  v.alt_m = 27.4f;
  v.vz_mps = 1.6f;
  v.state = "ASCENT";
  v.mode = "LIFT";
  v.alt_src = "BARO";
  OledDrawFlight(&v);
  save("flight_lift");
  v.alt_m = 29871.0f;
  v.vz_mps = -38.2f;
  v.state = "DESCENT";
  v.mode = "SIM";
  v.alt_src = "HOLD";  /* the widest label next to the widest altitude */
  OledDrawFlight(&v);
  save("flight_high");
  v.mode = NULL;
  v.state = "PRELAUNCH";

  OledDrawGps(&v);
  save("gps_no_data");
  v.gps_link = 1;
  v.gps_airborne = 1;
  OledDrawGps(&v);
  save("gps_searching");
  v.gps_fix = 1;
  v.sats_used = 14;
  v.sats_seen = 31;
  v.seen[0] = 11;
  v.seen[1] = 7;
  v.seen[2] = 8;
  v.seen[3] = 5;
  strcpy(v.utc, "085219");
  OledDrawGps(&v);
  save("gps_fix");

  v.radio_on = 1;
  v.tx_dbm = -9;
  v.freq_khz = 434500;
  v.sent = 109;
  v.board_temp_ok = v.out_temp_ok = 1;
  v.board_temp_c = 29.0f;
  v.out_temp_c = -62.4f;
  v.acc_g = 1.0f;
  v.sensors_ok = 5;
  v.sensors_all = 5;
  v.log_free_pct = 98;
  v.log_min_left = 432;
  v.uptime_s = 3 * 3600 + 12 * 60 + 33;
  OledDrawSystem(&v);
  save("system");

  /* running line: it moves, comes back to the start, and one step fits one frame's budget */
  OledDrawTicker(&v, 30);
  save("ticker");
  uint8_t first[sizeof(oled_fb)];
  int period = (OledTextWidth(OLED_TICKER_TEXT, 3) + 39) / OLED_TICKER_STEP;
  OledDrawTicker(&v, 5);
  memcpy(first, oled_fb, sizeof(first));
  OledDrawTicker(&v, 6);
  check(memcmp(first, oled_fb, sizeof(first)) != 0, "the line moves every frame");
  OledDrawTicker(&v, 5 + (uint32_t)period);
  check(memcmp(first, oled_fb, sizeof(first)) == 0, "and comes round again");
  OledFlush(100000);
  uint32_t worst = 0;
  for (uint32_t t = 0; t < (uint32_t)period; t++)
  {
    OledDrawTicker(&v, 6 + (uint32_t)period + t);
    n = OledFlush(400);
    worst = n > worst ? n : worst;
    check(memcmp(oled_fb, oled_shown, sizeof(oled_fb)) == 0, "a running-line frame is on screen within the same frame");
  }
  printf("running line: at most %lu bytes per frame (about %lu ms on the wire)\n", (unsigned long)worst,
         (unsigned long)(worst * 9U / 100U));

  printf(fails == 0 ? "oled: all checks passed\n" : "oled: %d FAILED\n", fails);
  return fails != 0;
}
