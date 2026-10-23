/* Altitude from the barometer and the GPS together (HW-15).
 *
 * Pure C, no HAL: included from main.c and from the host test tools/test_altitude.c.
 *
 * The BMP390 is specified down to 300 hPa, about 9 km. A balloon goes to 30 km, so above
 * that the altitude has to come from the GPS (Airborne <4g model: fine up there). The two
 * are blended by the measured pressure, not by altitude: above 400 hPa (below ~7.2 km)
 * only the barometer, below 320 hPa (above ~8.7 km) only the GPS, in between a linear mix.
 * The result never jumps:
 *   - the barometer and the GPS disagree by tens of metres (weather). During the flight,
 *     while the barometer is still good, that difference is learned slowly and added to
 *     the barometer, so both agree when the mix starts;
 *   - the GPS updates once a second, the frames come 5 times a second, and each sample
 *     is ~5 m off: the GPS altitude is followed by a small tracking filter (position and
 *     climb rate) that is carried forward between samples and moves 40 % of the way to
 *     each new one;
 *   - a GPS sample far from where the payload can be is ignored (5 in a row are accepted:
 *     then it is the payload that moved, after a long gap);
 *   - no GPS up high: the altitude is carried forward for 30 s (HOLD), then it is "not
 *     known" and the state machine waits instead of guessing.
 */
#ifndef ALTITUDE_FUSE_H
#define ALTITUDE_FUSE_H

#include <math.h>
#include <stdint.h>

#define ALT_SRC_BARO 0U   /* barometer only (also the value in older logs and packets) */
#define ALT_SRC_MIX  1U   /* both, 7-9 km */
#define ALT_SRC_GPS  2U   /* GPS only */
#define ALT_SRC_HOLD 3U   /* GPS lost up high: carried forward, at most 30 s */
#define ALT_SRC_NONE 4U   /* not known */

static const char *const ALT_SRC_NAME[] = {"BARO", "MIX", "GPS", "HOLD", "NONE"};

#define ALT_P_BARO_PA     40000.0  /* above this pressure: barometer only */
#define ALT_P_GPS_PA      32000.0  /* below this: GPS only (BMP390 spec ends at 30000 Pa) */
#define ALT_GPS_MIN_SATS  5        /* fewer satellites: the GPS altitude is not trusted */
#define ALT_BIAS_MIN_SATS 6        /* learning the weather difference needs a good fix */
#define ALT_BIAS_GAIN     0.02     /* per GPS sample (1 s): ~50 s to follow a change */
#define ALT_GPS_FRESH_MS  3000U    /* a GPS sample older than this is not used */
#define ALT_HOLD_MS       30000U   /* carry the altitude forward this long without GPS */
#define ALT_SPIKE_M       100.0    /* a GPS sample this far (+20 m per second of gap) from */
#define ALT_SPIKE_M_PER_S 20.0     /* where it should be is a glitch */
#define ALT_SPIKE_RESYNC  5        /* ... unless it happens this many times in a row */
#define ALT_GPS_POS_GAIN  0.4      /* tracking filter: share of each GPS sample in the altitude */
#define ALT_GPS_VZ_GAIN   0.3      /* ... and of its climb rate in the speed */

typedef struct
{
  int baro_ok;
  double press_pa;     /* measured pressure */
  double baro_asl_m;   /* altitude above sea level from that pressure */
  int gps_ok;          /* the GPS has a fix */
  int gps_sats;
  double gps_asl_m;
  uint32_t gps_ms;     /* when that GPS sample arrived: a new value means a new sample */
} AltInput;

typedef struct
{
  double asl_m;        /* output: altitude above sea level */
  float vz;            /* vertical speed, m/s, used to carry the altitude forward */
  uint8_t src;         /* ALT_SRC_* of the output */
  int ok;              /* output is usable */
  double bias_m;       /* GPS minus barometer, learned in flight below ~7 km */
  /* internal */
  uint32_t last_ms;
  uint32_t seen_gps_ms;  /* gps_ms of the last sample looked at */
  int gps_have;          /* an accepted GPS sample exists */
  double gps_raw;        /* last accepted GPS altitude as the GPS gave it */
  double gps_asl;        /* tracking filter: smoothed GPS altitude at gps_acc_ms ... */
  float gps_vz;          /* ... and climb rate from the GPS alone */
  uint32_t gps_acc_ms;   /* when the last sample was accepted */
  int spikes;            /* GPS samples rejected in a row */
  uint32_t fresh_ms;     /* last time the output came from a real measurement */
} AltFuse;

static inline void AltFuseInit(AltFuse *a)
{
  a->asl_m = 0.0;
  a->vz = 0.0f;
  a->src = ALT_SRC_NONE;
  a->ok = 0;
  a->bias_m = 0.0;
  a->last_ms = 0;
  a->seen_gps_ms = 0;
  a->gps_have = 0;
  a->gps_raw = 0.0;
  a->gps_asl = 0.0;
  a->gps_vz = 0.0f;
  a->gps_acc_ms = 0;
  a->spikes = 0;
  a->fresh_ms = 0;
}

/* Share of the barometer in the result: 1 below ~7 km, 0 above ~9 km or without it. */
static inline double AltBaroWeight(const AltInput *in)
{
  if (!in->baro_ok || in->press_pa <= ALT_P_GPS_PA)
  {
    return 0.0;
  }
  if (in->press_pa >= ALT_P_BARO_PA)
  {
    return 1.0;
  }
  return (in->press_pa - ALT_P_GPS_PA) / (ALT_P_BARO_PA - ALT_P_GPS_PA);
}

/* A new GPS sample: keep it, or drop it as a glitch. */
static inline void AltTakeGps(AltFuse *a, uint32_t now_ms, const AltInput *in, int learn)
{
  a->seen_gps_ms = in->gps_ms;
  if (!in->gps_ok || in->gps_sats < ALT_GPS_MIN_SATS)
  {
    return;
  }
  double g = in->gps_asl_m;
  if (a->gps_have && now_ms - a->gps_acc_ms < 10000U)
  {
    double dt = (now_ms - a->gps_acc_ms) / 1000.0;
    if (dt < 0.05)
    {
      dt = 0.05;
    }
    double expect = a->gps_raw + a->gps_vz * dt;
    if (fabs(g - expect) > ALT_SPIKE_M + ALT_SPIKE_M_PER_S * dt && ++a->spikes < ALT_SPIKE_RESYNC)
    {
      return;
    }
    double pred = a->gps_asl + a->gps_vz * dt;
    a->gps_vz += (float)ALT_GPS_VZ_GAIN * ((float)((g - a->gps_raw) / dt) - a->gps_vz);
    a->gps_asl = pred + ALT_GPS_POS_GAIN * (g - pred);
  }
  else
  {
    /* first sample, or the first after a long gap: start from it */
    a->gps_asl = g;
    a->gps_vz = a->vz;
  }
  a->spikes = 0;
  a->gps_have = 1;
  a->gps_raw = g;
  a->gps_acc_ms = now_ms;
  if (learn && in->baro_ok && in->press_pa >= ALT_P_BARO_PA && in->gps_sats >= ALT_BIAS_MIN_SATS)
  {
    a->bias_m += ALT_BIAS_GAIN * ((g - in->baro_asl_m) - a->bias_m);
  }
}

/* One frame. learn = 0 before the launch: the weather difference is then already taken
   care of by the sea-level calibration, and the launch altitude must stay fixed.
   Returns a->ok; the result is in a->asl_m and a->src. */
static inline int AltFuseUpdate(AltFuse *a, uint32_t now_ms, const AltInput *in, int learn)
{
  if (!learn)
  {
    a->bias_m = 0.0;
  }
  if (in->gps_ms != a->seen_gps_ms)
  {
    AltTakeGps(a, now_ms, in, learn);
  }
  double dt = a->ok ? (now_ms - a->last_ms) / 1000.0 : 0.0;
  double prev = a->asl_m;
  int fresh_gps = a->gps_have && now_ms - a->gps_acc_ms <= ALT_GPS_FRESH_MS;
  double gps_now = a->gps_asl + a->gps_vz * ((now_ms - a->gps_acc_ms) / 1000.0);
  double w = AltBaroWeight(in);
  double baro_now = in->baro_asl_m + a->bias_m;

  if (w >= 1.0 || (w > 0.0 && !fresh_gps))
  {
    a->asl_m = baro_now;
    a->src = ALT_SRC_BARO;
  }
  else if (w > 0.0)
  {
    a->asl_m = w * baro_now + (1.0 - w) * gps_now;
    a->src = ALT_SRC_MIX;
  }
  else if (fresh_gps)
  {
    a->asl_m = gps_now;
    a->src = ALT_SRC_GPS;
  }
  else if (a->ok && now_ms - a->fresh_ms <= ALT_HOLD_MS)
  {
    a->asl_m = prev + a->vz * dt;
    a->src = ALT_SRC_HOLD;
  }
  else
  {
    a->src = ALT_SRC_NONE;
    a->ok = 0;
    a->last_ms = now_ms;
    return 0;
  }

  if (a->src != ALT_SRC_HOLD)
  {
    a->fresh_ms = now_ms;
  }
  if (a->src == ALT_SRC_GPS)
  {
    a->vz = a->gps_vz;  /* only the GPS sees the payload now: its climb rate is the speed */
  }
  else if ((a->src == ALT_SRC_BARO || a->src == ALT_SRC_MIX) && a->ok && dt > 0.0)
  {
    a->vz += 0.1f * ((float)((a->asl_m - prev) / dt) - a->vz);
  }
  else if (!a->ok)
  {
    a->vz = 0.0f;
  }
  a->ok = 1;
  a->last_ms = now_ms;
  return 1;
}

/* Inverse of IsaAltitude (flight_sm.h): pressure of the standard atmosphere at an
   altitude. Used by the 'S' simulation and the host test to make barometer readings. */
static inline double IsaPressure(double h_m)
{
  if (h_m < 11000.0)
  {
    return 101325.0 * pow(1.0 - h_m / 44330.77, 1.0 / 0.190263);
  }
  if (h_m < 20000.0)
  {
    return 22632.06 * exp(-(h_m - 11000.0) / 6341.62);
  }
  if (h_m < 32000.0)
  {
    return 5474.89 * pow(1.0 + (h_m - 20000.0) / 216650.0, -1.0 / 0.0292712);
  }
  return 868.02 * pow(1.0 + (h_m - 32000.0) / 81660.7, -1.0 / 0.0819591);
}

#endif /* ALTITUDE_FUSE_H */
