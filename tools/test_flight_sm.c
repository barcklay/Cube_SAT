/* Host test of the flight state machine (HW-18): flies synthetic balloons through
 * hab_bringup/Core/Inc/flight_sm.h on the laptop.
 *
 *     cc -O2 -o /tmp/test_flight_sm tools/test_flight_sm.c -lm && /tmp/test_flight_sm
 *
 * Each scenario is a profile of true altitude above the launch point; the test adds
 * barometer noise and the swing of the payload, feeds 5 frames per second and checks
 * which states were reached and when.
 */
#include <stdio.h>
#include <stdlib.h>
#include "../hab_bringup/Core/Inc/flight_sm.h"

#define DT_MS 200U

static double noise(double sigma)  /* Gaussian, Box-Muller */
{
  double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
  return sigma * sqrt(-2.0 * log(u)) * cos(6.283185307 * v);
}

/* Parachute descent speed: 5 m/s at the ground, faster where the air is thin. */
static double descent_speed(double h) { return 5.0 * exp(h / 14600.0); }

typedef struct
{
  const char *name;
  double pre_s, ascent_vz, burst_m, float_s, leak_vz, landed_s;
  double sigma;          /* barometer noise, m */
  double lift_m, lift_vz; /* a lift ride on the ground before the launch */
  double gap_from_s, gap_to_s;  /* altitude source lost */
  double reset_at_s;     /* board reset in flight (resume from the log) */
  FlightState expect_final;
  int expect_burst, expect_float;
  double stop_every_m;   /* ascent pauses (turbulence / a lift stopping at floors) */
} Scenario;

static int run(const Scenario *s)
{
  FlightSm sm;
  FlightSmInit(&sm, 0);
  double h = 0, t = 0, burst_t = -1, land_t = -1, float_until = -1;
  int phase = 0;  /* 0 ground, 1 lift, 2 wait, 3 up, 4 float, 5 down, 6 ground */
  double lift_done = 0;
  uint32_t seen[6] = {0};
  int reset_done = 0;
  double next_stop = s->stop_every_m, pause_until = -1;
  double end_s = s->pre_s > 1e8 ? 8 * 3600.0 : s->pre_s + 30000;
  for (uint32_t ms = 0; t < end_s; ms += DT_MS, t = ms / 1000.0)
  {
    /* true altitude */
    if (phase == 0 && s->lift_m > 0 && t > 60) phase = 1;
    if (phase == 1) { h += s->lift_vz * DT_MS / 1000.0; if (h >= s->lift_m) { h = s->lift_m; phase = 2; lift_done = t; } }
    if ((phase == 0 || phase == 2) && t >= s->pre_s) { phase = 3; h = s->lift_m; }
    if (phase == 3)
    {
      if (t >= pause_until)
      {
        h += s->ascent_vz * DT_MS / 1000.0;
        if (s->stop_every_m > 0 && h >= next_stop && h < 2000)
        {
          pause_until = t + 15;  /* stand still 15 s every stop_every_m metres */
          next_stop += s->stop_every_m;
        }
      }
      if (h >= s->burst_m)
      {
        h = s->burst_m;
        if (s->float_s > 0) { phase = 4; float_until = t + s->float_s; }
        else { phase = 5; burst_t = t; }
      }
    }
    if (phase == 4 && t >= float_until) { phase = 5; burst_t = t; }
    if (phase == 5)
    {
      double v = s->leak_vz > 0 ? s->leak_vz : descent_speed(h);
      h -= v * DT_MS / 1000.0;
      if (h <= 0) { h = 0; phase = 6; land_t = t; }
    }
    if (phase == 6 && t > land_t + s->landed_s) break;
    (void)lift_done;

    /* reset in flight: the firmware resumes state and max altitude from the log */
    if (s->reset_at_s > 0 && !reset_done && t >= s->reset_at_s)
    {
      FlightSmResume(&sm, ms, sm.state, sm.max_alt);
      reset_done = 1;
    }

    double swing = (phase >= 3 && phase <= 5) ? 1.5 * sin(t * 1.3) : 0.0;
    int ok = !(t >= s->gap_from_s && t < s->gap_to_s);
    FlightState before = sm.state;
    if (FlightSmUpdate(&sm, ms, (float)(h + noise(s->sigma) + swing), ok))
    {
      seen[sm.state] = ms ? ms : 1;
      printf("    t=%7.1f s  %-9s -> %-9s  h=%8.1f  vz=%+6.1f\n", t, FS_NAME[before], FS_NAME[sm.state],
             sm.alt, sm.vz);
    }
  }

  int ok = sm.state == s->expect_final;
  ok &= (seen[FS_BURST] != 0) == s->expect_burst;
  ok &= (seen[FS_FLOAT] != 0) == s->expect_float;
  if (s->expect_final != FS_PRELAUNCH)
  {
    ok &= seen[FS_ASCENT] != 0;
  }
  if (seen[FS_BURST] && burst_t >= 0)
  {
    double delay = seen[FS_BURST] / 1000.0 - burst_t;
    ok &= delay >= 0 && delay < 10;
    printf("    burst detected %.1f s after it happened\n", delay);
  }
  if (seen[FS_LANDED] && land_t >= 0)
  {
    double delay = seen[FS_LANDED] / 1000.0 - land_t;
    ok &= delay >= 0 && delay < 90;
    printf("    landing detected %.1f s after touchdown\n", delay);
  }
  printf("%s  %s (final %s)\n\n", ok ? "PASS" : "FAIL", s->name, FS_NAME[sm.state]);
  return ok;
}

int main(void)
{
  srand(42);
  const Scenario sc[] = {
    {"normal flight, 30 km burst", 600, 5, 30000, 0, 0, 600, 1.0, 0, 0, -1, -1, 0, FS_LANDED, 1, 0},
    {"noisy barometer (3 m)", 600, 5, 30000, 0, 0, 600, 3.0, 0, 0, -1, -1, 0, FS_LANDED, 1, 0},
    {"slow leak, no burst", 600, 5, 20000, 0, 2.0, 600, 1.0, 0, 0, -1, -1, 0, FS_LANDED, 0, 0},
    {"float 3 h at 25 km, then burst", 600, 5, 25000, 3 * 3600, 0, 600, 1.0, 0, 0, -1, -1, 0, FS_LANDED, 1, 1},
    {"lift to the 21st floor before launch", 900, 5, 30000, 0, 0, 600, 1.0, 65, 2.5, -1, -1, 0, FS_LANDED, 1, 0},
    {"barometer lost for 2 min at 12 km", 600, 5, 30000, 0, 0, 600, 1.0, 0, 0, 3000, 3120, 0, FS_LANDED, 1, 0},
    {"reset during the descent", 600, 5, 30000, 0, 0, 600, 1.0, 0, 0, -1, -1, 6900, FS_LANDED, 1, 0},
    {"on the bench for 8 h (never launched)", 1e9, 5, 30000, 0, 0, 0, 1.0, 0, 0, -1, -1, 0, FS_PRELAUNCH, 0, 0},
    {"ascent with pauses every 60 m", 600, 5, 30000, 0, 0, 600, 1.0, 0, 0, -1, -1, 0, FS_LANDED, 1, 0, 60},
    {"stop-and-go every 20 m (like a lift)", 600, 5, 30000, 0, 0, 600, 1.0, 0, 0, -1, -1, 0, FS_LANDED, 1, 0, 20},
  };
  int pass = 0, n = sizeof(sc) / sizeof(sc[0]);
  for (int i = 0; i < n; i++)
  {
    printf("[%d] %s\n", i + 1, sc[i].name);
    pass += run(&sc[i]);
  }
  printf("ISA altitude check: 101325 Pa -> %.0f m, 22632 Pa -> %.0f m, 1197 Pa -> %.0f m\n",
         IsaAltitude(101325), IsaAltitude(22632.06), IsaAltitude(1197));
  printf("%d/%d scenarios passed\n", pass, n);
  return pass == n ? 0 : 1;
}
