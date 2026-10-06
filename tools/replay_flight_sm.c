/* Replay a recorded ride through the flight state machine in lift-test mode.

   Takes the black-box pressure of a real lift ride and feeds it to the same code the board
   runs (Core/Inc/flight_sm.h), as if 'T' had been sent at the bottom floor.

       python3 tools/logdump.py ride.csv                       # download the black box
       awk -F, '$2==37{print $4","$6}' ride.csv > ride37.csv   # one boot: t_ms,baro_pa
       cc -O2 -I hab_bringup/Core/Inc -o /tmp/replay tools/replay_flight_sm.c -lm
       /tmp/replay ride37.csv 185                              # 185 = second when 'T' is sent
*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "flight_sm.h"
int main(int argc, char **argv)
{
  FILE *f = fopen(argv[1], "r");
  double t_arm = atof(argv[2]);  /* s: when 'T' is sent (20 s after arriving at the bottom) */
  FlightSm sm; double ground = 0; int armed = 0; unsigned t; double p; int last = -1;
  while (fscanf(f, "%u,%lf", &t, &p) == 2)
  {
    if (!armed && t / 1000.0 >= t_arm)
    {
      armed = 1; ground = p; FlightSmInit(&sm, t); sm.scale = 0.05f; sm.landed_max = 10.0f;
      printf("%6.1f s  T sent: test mode ON, ground = this floor\n", t / 1000.0);
    }
    if (!armed) continue;
    double alt = IsaAltitude(p) - IsaAltitude(ground);
    FlightSmUpdate(&sm, t, (float)alt, 1);
    if ((int)sm.state != last)
    {
      last = sm.state;
      printf("%6.1f s  -> %-9s h=%6.1f m  max=%5.1f m\n", t / 1000.0, FS_NAME[sm.state], alt, sm.max_alt);
    }
  }
  return 0;
}
