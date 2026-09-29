/* Flight state machine and standard-atmosphere altitude (HW-18).
 *
 * Pure C, no HAL: the same file is compiled into the firmware (included from main.c)
 * and into the host test tools/test_flight_sm.c, which flies synthetic balloons
 * through it on the laptop.
 *
 * Input: altitude above the launch point (m) once per frame. Everything is decided
 * from altitude and vertical speed only, so a long horizontal drift or a float at
 * constant altitude never looks like a landing.
 */
#ifndef FLIGHT_SM_H
#define FLIGHT_SM_H

#include <math.h>
#include <stdint.h>

typedef enum
{
  FS_PRELAUNCH = 0,  /* on the ground before the launch */
  FS_ASCENT,         /* going up under the balloon */
  FS_FLOAT,          /* balloon stopped rising (not burst): drifting at altitude */
  FS_BURST,          /* the balloon just burst: first 60 s of the fall */
  FS_DESCENT,        /* coming down under the parachute */
  FS_LANDED          /* on the ground again: beep, log slowly */
} FlightState;

static const char *const FS_NAME[] = {"PRELAUNCH", "ASCENT", "FLOAT", "BURST", "DESCENT", "LANDED"};

/* Thresholds (m, m/s, ms). A condition must hold for its time before the state changes,
   so one noisy sample or the payload swinging under the balloon changes nothing. */
#define FS_LAUNCH_ALT_M      300.0f   /* higher than any lift or stairs */
#define FS_LAUNCH_VZ         1.5f     /* the balloon rises at ~5 m/s */
#define FS_LAUNCH_HOLD_MS    10000U
#define FS_BURST_VZ          (-8.0f)  /* after a burst the payload falls at 30-40 m/s */
#define FS_BURST_HOLD_MS     3000U
#define FS_LEAK_DROP_M       500.0f   /* slow leak: 500 m below the highest point */
#define FS_BURST_SHOW_MS     60000U   /* BURST is shown for 60 s, then DESCENT */
#define FS_STILL_VZ          1.0f     /* "not moving": to start watching the altitude range */
#define FS_STILL_RANGE_M     25.0f    /* landed: altitude stays within 25 m for the hold time */
#define FS_FLOAT_RANGE_M     150.0f   /* float: within 150 m for 5 min (a floating balloon bobs) */
#define FS_FLOAT_MIN_ALT_M   1000.0f
#define FS_FLOAT_HOLD_MS     300000U  /* 5 min without climbing at altitude */
#define FS_REASCENT_HOLD_MS  30000U
#define FS_LANDED_HOLD_MS    60000U
#define FS_LANDED_MAX_ALT_M  3000.0f  /* cannot have landed 3 km above the launch point */

#define FS_WINDOW 25  /* vertical speed: slope over the last 25 frames = 5 s at 5 Hz */

typedef struct
{
  FlightState state;
  uint32_t state_ms;   /* when the current state began */
  float max_alt;       /* highest altitude above launch seen so far */
  float alt;           /* last altitude above launch */
  float vz;            /* vertical speed, m/s, + = up */
  int vz_ok;           /* enough samples for vz */
  uint32_t hold_a_ms;  /* since when condition A holds (0 = not holding) */
  uint32_t hold_b_ms;  /* since when condition B holds */
  float still_min, still_max;  /* altitude range since the "not moving" watch started */
  float scale;         /* altitude thresholds x scale: 1 = flight, 0.05 = lift test */
  float landed_max;    /* landing counts only below this altitude above launch */
  uint32_t win_t[FS_WINDOW];
  float win_h[FS_WINDOW];
  int win_n, win_head;
} FlightSm;

/* Pressure altitude of the standard atmosphere (ISA), layered: 0-11 km lapse rate,
   11-20 km isothermal, 20-32 km and 32-47 km warming. The single-layer formula
   44330*(1-(p/p0)^0.19) is only right below 11 km and is ~4.7 km low at 30 km. */
static inline double IsaAltitude(double p_pa)
{
  if (p_pa > 22632.06)
  {
    return 44330.77 * (1.0 - pow(p_pa / 101325.0, 0.190263));
  }
  if (p_pa > 5474.89)
  {
    return 11000.0 + 6341.62 * log(22632.06 / p_pa);
  }
  if (p_pa > 868.02)
  {
    return 20000.0 + 216650.0 * (pow(p_pa / 5474.89, -0.0292712) - 1.0);
  }
  return 32000.0 + 81660.7 * (pow(p_pa / 868.02, -0.0819591) - 1.0);
}

static inline void FlightSmInit(FlightSm *sm, uint32_t now_ms)
{
  sm->state = FS_PRELAUNCH;
  sm->state_ms = now_ms;
  sm->max_alt = 0.0f;
  sm->alt = 0.0f;
  sm->vz = 0.0f;
  sm->vz_ok = 0;
  sm->hold_a_ms = sm->hold_b_ms = 0;
  sm->win_n = sm->win_head = 0;
  sm->scale = 1.0f;
  sm->landed_max = FS_LANDED_MAX_ALT_M;
}

/* After a reset in flight: continue from the state saved in the log. */
static inline void FlightSmResume(FlightSm *sm, uint32_t now_ms, FlightState state, float max_alt)
{
  FlightSmInit(sm, now_ms);
  sm->state = state;
  sm->max_alt = max_alt;
}

/* Least-squares slope of altitude over time in the window (m/s). */
static inline void FlightSmVz(FlightSm *sm)
{
  if (sm->win_n < 10)
  {
    sm->vz_ok = 0;
    return;
  }
  int oldest = (sm->win_head - sm->win_n + FS_WINDOW) % FS_WINDOW;
  uint32_t t0 = sm->win_t[oldest];
  double st = 0, sh = 0, stt = 0, sth = 0;
  for (int i = 0; i < sm->win_n; i++)
  {
    int k = (oldest + i) % FS_WINDOW;
    double t = (sm->win_t[k] - t0) / 1000.0, h = sm->win_h[k];
    st += t; sh += h; stt += t * t; sth += t * h;
  }
  double n = sm->win_n, den = n * stt - st * st;
  if (den <= 1e-6)
  {
    sm->vz_ok = 0;
    return;
  }
  sm->vz = (float)((n * sth - st * sh) / den);
  sm->vz_ok = 1;
}

/* 1 when `cond` has been true for at least `hold_ms` (tracked in *since). */
static inline int FlightSmHeld(uint32_t *since, int cond, uint32_t now_ms, uint32_t hold_ms)
{
  if (!cond)
  {
    *since = 0;
    return 0;
  }
  if (*since == 0)
  {
    *since = now_ms ? now_ms : 1;
  }
  return now_ms - *since >= hold_ms;
}

/* "Not moving": starts when |vz| is small, then only the altitude range counts, so a
   swinging payload or barometer noise cannot restart the wait again and again. */
static inline int FlightSmStill(FlightSm *sm, uint32_t *since, float h, float vz, uint32_t now_ms,
                                uint32_t hold_ms, float max_range)
{
  if (*since == 0)
  {
    if (fabsf(vz) >= FS_STILL_VZ)
    {
      return 0;
    }
    *since = now_ms ? now_ms : 1;
    sm->still_min = sm->still_max = h;
  }
  if (h < sm->still_min) sm->still_min = h;
  if (h > sm->still_max) sm->still_max = h;
  if (sm->still_max - sm->still_min > max_range)
  {
    *since = 0;  /* it moved: start watching again */
    return 0;
  }
  return now_ms - *since >= hold_ms;
}

static inline void FlightSmGo(FlightSm *sm, FlightState s, uint32_t now_ms)
{
  sm->state = s;
  sm->state_ms = now_ms;
  sm->hold_a_ms = sm->hold_b_ms = 0;
}

/* One frame. alt_ok = 0 when no altitude source works: the state is then frozen.
   Returns 1 when the state changed. */
static inline int FlightSmUpdate(FlightSm *sm, uint32_t now_ms, float alt_m, int alt_ok)
{
  FlightState before = sm->state;
  if (alt_ok)
  {
    sm->alt = alt_m;
    sm->win_t[sm->win_head] = now_ms;
    sm->win_h[sm->win_head] = alt_m;
    sm->win_head = (sm->win_head + 1) % FS_WINDOW;
    if (sm->win_n < FS_WINDOW)
    {
      sm->win_n++;
    }
    FlightSmVz(sm);
    if (sm->state != FS_PRELAUNCH && sm->state != FS_LANDED && alt_m > sm->max_alt)
    {
      sm->max_alt = alt_m;
    }
  }
  if (!alt_ok || !sm->vz_ok)
  {
    return 0;
  }
  float h = sm->alt, vz = sm->vz;

  switch (sm->state)
  {
    case FS_PRELAUNCH:
      /* Climbing steadily above 300 m, or simply 600 m up: a climb with pauses (turbulence,
         or a lift stopping at floors in the lift test) must still count as a launch. */
      if (FlightSmHeld(&sm->hold_a_ms, h > FS_LAUNCH_ALT_M * sm->scale && vz > FS_LAUNCH_VZ, now_ms, FS_LAUNCH_HOLD_MS) ||
          h > 2.0f * FS_LAUNCH_ALT_M * sm->scale)
      {
        FlightSmGo(sm, FS_ASCENT, now_ms);
        sm->max_alt = h;
      }
      break;

    case FS_ASCENT:
    case FS_FLOAT:
      if (FlightSmHeld(&sm->hold_a_ms, vz < FS_BURST_VZ, now_ms, FS_BURST_HOLD_MS))
      {
        FlightSmGo(sm, FS_BURST, now_ms);           /* fast fall: the balloon burst */
      }
      else if (h < sm->max_alt - FS_LEAK_DROP_M * sm->scale)
      {
        /* 500 m below the top: falling fast = burst, slowly = leak (no burst) */
        FlightSmGo(sm, vz < FS_BURST_VZ ? FS_BURST : FS_DESCENT, now_ms);
      }
      else if (sm->state == FS_ASCENT &&
               h > FS_FLOAT_MIN_ALT_M * sm->scale &&
               FlightSmStill(sm, &sm->hold_b_ms, h, vz, now_ms, FS_FLOAT_HOLD_MS, FS_FLOAT_RANGE_M * sm->scale))
      {
        FlightSmGo(sm, FS_FLOAT, now_ms);
      }
      else if (sm->state == FS_FLOAT &&
               FlightSmHeld(&sm->hold_b_ms, vz > FS_LAUNCH_VZ, now_ms, FS_REASCENT_HOLD_MS))
      {
        FlightSmGo(sm, FS_ASCENT, now_ms);          /* started climbing again */
      }
      break;

    case FS_BURST:
      if (now_ms - sm->state_ms >= FS_BURST_SHOW_MS)
      {
        FlightSmGo(sm, FS_DESCENT, now_ms);
      }
      break;

    case FS_DESCENT:
      if (h < sm->landed_max &&
          FlightSmStill(sm, &sm->hold_a_ms, h, vz, now_ms, FS_LANDED_HOLD_MS, FS_STILL_RANGE_M))
      {
        FlightSmGo(sm, FS_LANDED, now_ms);
      }
      break;

    case FS_LANDED:
      break;
  }
  return sm->state != before;
}

#endif /* FLIGHT_SM_H */
