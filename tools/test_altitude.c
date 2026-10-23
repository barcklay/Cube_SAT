/* Host test of the barometer + GPS altitude (HW-15): flies synthetic balloons to 33 km
 * through hab_bringup/Core/Inc/altitude_fuse.h and the flight state machine.
 *
 *     cc -O2 -Wall -o /tmp/test_altitude tools/test_altitude.c -lm && /tmp/test_altitude
 *
 * The barometer is made from the standard atmosphere plus a weather error that grows
 * with altitude (a real day is never standard) and stops being trusted below 320 hPa;
 * the GPS gives one sample a second with 5 m of noise, and can be switched off or given
 * a glitch at chosen altitudes.
 */
#include <stdio.h>
#include <stdlib.h>
#include "../hab_bringup/Core/Inc/flight_sm.h"
#include "../hab_bringup/Core/Inc/altitude_fuse.h"

#define DT_MS 200U
#define LAUNCH_ASL 150.0
#ifndef SEED
#define SEED 1  /* -DSEED=n: the same flights with other noise */
#endif

static int fails;

static void check(int cond, const char *name, const char *what)
{
  if (!cond)
  {
    fails++;
    printf("FAIL [%s]: %s\n", name, what);
  }
}

static double noise(double sigma)  /* Gaussian, Box-Muller */
{
  double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
  return sigma * sqrt(-2.0 * log(u)) * cos(6.283185307 * v);
}

typedef struct
{
  const char *name;
  int gps_off;              /* no GPS at all */
  int baro_off;             /* no barometer at all */
  double calib_err_m;       /* barometer sea level not calibrated by GPS before launch */
  double gap1_m, gap1_s;    /* GPS lost on the way up at this altitude for this long */
  double gap2_m, gap2_s;
  double spike_m;           /* one GPS sample 800 m off at this altitude */
} Scenario;

static void run(const Scenario *s)
{
  const double pre_s = 600.0, top_m = 33000.0, scale_m = 14600.0;
  const double up_s = top_m / 5.0;
  FlightSm sm;
  AltFuse af;
  FlightSmInit(&sm, 0);
  AltFuseInit(&af);
  double h = 0.0, prev_out = 0.0, prev_h = 0.0;
  int prev_ok = 0;
  double max_err = 0.0, max_step = 0.0, mix_step = 0.0, spike_step = 0.0;
  double burst_err = 0.0, burst_step = 0.0, gap_step = 0.0, back_t = -1;
  uint8_t prev_src = ALT_SRC_NONE;
  int seen[5] = {0};
  double burst_true_s = pre_s + up_s, burst_seen_s = -1, land_true_s = -1, land_seen_s = -1;
  int early = 0, floated = 0;
  double gap1_from = -1, gap2_from = -1, spike_t = -1;
  uint32_t gps_ms = 0;
  double gps_asl = 0.0;
  int gps_fix = 0;
  double launch_asl = 0.0;

  for (uint32_t ms = 0;; ms += DT_MS)
  {
    double t = ms / 1000.0;
    /* true altitude above the launch point */
    if (t < pre_s) h = 0.0;
    else if (t < pre_s + up_s) h = (t - pre_s) * 5.0;
    else
    {
      double e = exp(-top_m / scale_m) + 5.0 * (t - pre_s - up_s) / scale_m;
      h = e >= 1.0 ? 0.0 : -scale_m * log(e);
      if (h == 0.0 && land_true_s < 0) land_true_s = t;
    }
    if (land_true_s > 0 && t > land_true_s + 900.0) break;
    double asl = LAUNCH_ASL + h;
    int going_up = t >= pre_s && t < pre_s + up_s;

    /* barometer: weather error 0.006 m per m of height; the GPS calibration before the
       launch removes the ground part of it (calib_err_m = what is left without GPS) */
    double weather = 0.006 * h + s->calib_err_m;
    double p = IsaPressure(asl + weather + noise(0.3));
    AltInput in = {0};
    in.baro_ok = !s->baro_off;
    in.press_pa = p;
    in.baro_asl_m = IsaAltitude(p);

    /* GPS, once a second */
    if (gap1_from < 0 && going_up && h >= s->gap1_m) gap1_from = t;
    if (gap2_from < 0 && going_up && h >= s->gap2_m) gap2_from = t;
    int in_gap = (gap1_from >= 0 && t < gap1_from + s->gap1_s) || (gap2_from >= 0 && t < gap2_from + s->gap2_s);
    if (ms % 1000U == 0U && !s->gps_off && !in_gap)
    {
      gps_ms = ms + 1U;
      gps_fix = 1;
      gps_asl = asl + noise(5.0);
      if (spike_t < 0 && going_up && s->spike_m > 0 && h >= s->spike_m)
      {
        gps_asl += 800.0;
        spike_t = t;
      }
    }
    else if (ms % 1000U == 0U)
    {
      gps_fix = 0;
    }
    in.gps_ok = gps_fix;
    in.gps_sats = gps_fix ? 10 : 0;
    in.gps_asl_m = gps_asl;
    in.gps_ms = gps_ms;

    int ok = AltFuseUpdate(&af, ms, &in, sm.state != FS_PRELAUNCH);
    if (t < 30.0)
    {
      launch_asl = af.asl_m;  /* like ground_pa in the firmware: taken once, at power-on */
    }
    double out = af.asl_m - launch_asl;
    seen[af.src]++;
    FlightState before = sm.state;
    FlightSmUpdate(&sm, ms, (float)out, ok);
    if (sm.state != before)
    {
      if (sm.state == FS_FLOAT) floated = 1;
      if ((sm.state == FS_BURST || sm.state == FS_DESCENT) && burst_seen_s < 0)
      {
        burst_seen_s = t;
        if (t < burst_true_s) early = 1;
      }
      if (sm.state == FS_LANDED) land_seen_s = t;
    }

    /* Measured apart, not limited the same way: the 30 s after the burst (the speed goes
       from +5 to -48 m/s between two GPS samples, so the GPS lags for a few seconds) and
       the first 2 s after the GPS comes back from a gap (the carried-forward altitude is
       corrected to the new sample in one go). */
    int after_burst = t >= burst_true_s && t < burst_true_s + 30.0;
    if (ok && (prev_src == ALT_SRC_HOLD || prev_src == ALT_SRC_NONE) && af.src != ALT_SRC_HOLD)
    {
      back_t = t;
    }
    int after_gap = back_t >= 0 && t - back_t < 2.0;
    if (ok && t > 60.0)
    {
      double err = fabs(out - h);
      if (after_burst)
      {
        if (err > burst_err) burst_err = err;
      }
      else if (af.src != ALT_SRC_HOLD && !after_gap && err > max_err)
      {
        max_err = err;
      }
      if (prev_ok)
      {
        double step = fabs((out - prev_out) - (h - prev_h));
        if (after_burst && step > burst_step) burst_step = step;
        if (after_gap && step > gap_step) gap_step = step;
        if (!after_burst && !after_gap && step > max_step) max_step = step;
        if (af.src == ALT_SRC_MIX && step > mix_step) mix_step = step;
        if (spike_t >= 0 && t - spike_t < 1.5 && step > spike_step) spike_step = step;
      }
    }
    prev_ok = ok && t > 60.0;
    prev_src = af.src;
    prev_out = out;
    prev_h = h;
  }

  char what[160];
  printf("%s\n  error %.1f m, step %.1f m | after the burst: error %.1f m, step %.1f m | "
         "after a GPS gap: step %.1f m\n  burst seen +%.1f s, landing +%.1f s, top %.0f m | "
         "frames BARO %d MIX %d GPS %d HOLD %d NONE %d\n", s->name, max_err, max_step, burst_err,
         burst_step, gap_step, burst_seen_s - burst_true_s, land_seen_s - land_true_s, sm.max_alt,
         seen[0], seen[1], seen[2], seen[3], seen[4]);
  check(!early, s->name, "no burst before the real one");
  check(!floated, s->name, "no float on a steady climb");
  check(land_seen_s > 0 && land_seen_s - land_true_s < 180.0, s->name, "landing found within 3 min");
  if (s->gps_off)
  {
    /* without GPS the altitude is lost above ~8.7 km: the state machine waits, then the
       barometer finds the payload again on the way down */
    check(sm.max_alt > 8000.0 && sm.max_alt < 10000.0, s->name, "top = where the barometer stopped");
    check(seen[ALT_SRC_GPS] == 0 && seen[ALT_SRC_NONE] > 0, s->name, "no GPS: altitude unknown up high");
    check(burst_seen_s > 0, s->name, "the fall is noticed once the barometer is back");
    return;
  }
  snprintf(what, sizeof(what), "burst seen within 15 s (%.1f s)", burst_seen_s - burst_true_s);
  check(burst_seen_s > 0 && burst_seen_s - burst_true_s < 15.0, s->name, what);
  snprintf(what, sizeof(what), "top within 1 %% of 33 km (%.0f m)", sm.max_alt);
  check(fabs(sm.max_alt - top_m) < 330.0, s->name, what);
  /* without the GPS calibration before the launch the launch point itself is off by
     calib_err_m; the GPS part of the flight then shows that offset */
  snprintf(what, sizeof(what), "altitude within 60 m of the truth (%.1f m)", max_err);
  check(max_err < 60.0 + s->calib_err_m, s->name, what);
  snprintf(what, sizeof(what), "no jump from one frame to the next (%.1f m)", max_step);
  check(max_step < 25.0, s->name, what);
  if (!s->baro_off)
  {
    snprintf(what, sizeof(what), "smooth through the 7-9 km mix (%.1f m)", mix_step);
    check(seen[ALT_SRC_MIX] > 0 && mix_step < 10.0, s->name, what);
    check(seen[ALT_SRC_BARO] > 0 && seen[ALT_SRC_GPS] > 0, s->name, "barometer low, GPS high");
  }
  if (s->gap1_s > 0)
  {
    check(seen[ALT_SRC_HOLD] > 0, s->name, "GPS gap: altitude carried forward");
  }
  snprintf(what, sizeof(what), "right after the burst: within 120 m (%.1f m)", burst_err);
  check(burst_err < 120.0, s->name, what);
  if (s->gap2_s > ALT_HOLD_MS / 1000.0)
  {
    check(seen[ALT_SRC_NONE] > 0, s->name, "long GPS gap: altitude unknown, not guessed");
  }
  if (s->spike_m > 0)
  {
    snprintf(what, sizeof(what), "800 m GPS glitch ignored (%.1f m)", spike_step);
    check(spike_t > 0 && spike_step < 25.0, s->name, what);
  }
}

int main(void)
{
  srand(SEED);
  static const Scenario S[] = {
    {"clean flight", 0, 0, 0.0, 0, 0, 0, 0, 0},
    {"GPS gaps 20 s at 15 km, 2 min at 20 km", 0, 0, 0.0, 15000, 20, 20000, 120, 0},
    {"GPS glitch +800 m at 25 km", 0, 0, 0.0, 0, 0, 0, 0, 25000},
    {"sea level not calibrated (+80 m)", 0, 0, 80.0, 0, 0, 0, 0, 0},
    {"barometer dead", 0, 1, 0.0, 0, 0, 0, 0, 0},
    {"GPS dead", 1, 0, 0.0, 0, 0, 0, 0, 0},
  };
  for (unsigned i = 0; i < sizeof(S) / sizeof(S[0]); i++)
  {
    run(&S[i]);
  }

  /* the standard atmosphere and its inverse agree everywhere a balloon goes */
  double worst = 0.0;
  for (double hh = 0.0; hh <= 40000.0; hh += 250.0)
  {
    double e = fabs(IsaAltitude(IsaPressure(hh)) - hh);
    worst = e > worst ? e : worst;
  }
  check(worst < 0.01, "ISA", "IsaPressure is the inverse of IsaAltitude");

  printf(fails == 0 ? "altitude: all checks passed\n" : "altitude: %d FAILED\n", fails);
  return fails != 0;
}
