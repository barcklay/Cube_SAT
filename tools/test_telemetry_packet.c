/* Test of the radio packet (hab_bringup/Core/Inc/telemetry_packet.h) on the laptop:

       cc -O2 -Wall -I hab_bringup/Core/Inc -o /tmp/tp tools/test_telemetry_packet.c -lm && /tmp/tp
*/
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "telemetry_packet.h"

static int fails;
#define CHECK(cond, what) do { if (!(cond)) { printf("FAIL: %s\n", what); fails++; } } while (0)

int main(void)
{
  uint8_t b[TP_SIZE];
  Telemetry t = {0}, r;

  /* 1. the CRC is the standard CRC-16/CCITT-FALSE: "123456789" -> 0x29B1 */
  CHECK(TpCrc16((const uint8_t *)"123456789", 9) == 0x29B1, "CRC-16/CCITT-FALSE check value");

  /* 2. a typical packet in the stratosphere comes back the same */
  t.payload_id = 2; t.seq = 1234; t.uptime_s = 5400; t.state = 1;
  t.flags = TP_FLAG_BARO_OK | TP_FLAG_GPS_FIX | TP_FLAG_OUT_OK | TP_FLAG_LOG_OK; t.gps_sats = 14;
  t.lat_e7 = 158000000; t.lon_e7 = 1025000000; t.gps_alt_m = 29871.4f; t.alt_m = 29702.6f;
  t.vz_mps = 5.37f; t.out_temp_c = -62.4f; t.board_temp_c = -11.2f; t.vbat_v = 6.12f;
  TelemetryEncode(&t, b);
  CHECK(TelemetryDecode(b, TP_SIZE, &r) == 0, "decode of a good packet");
  CHECK(r.payload_id == 2 && r.seq == 1234 && r.uptime_s == 5400 && r.state == 1, "header fields");
  CHECK(r.flags == t.flags && r.gps_sats == 14, "flags and satellites");
  CHECK(r.lat_e7 == t.lat_e7 && r.lon_e7 == t.lon_e7, "position exact to 1e-7 degree");
  CHECK(fabsf(r.gps_alt_m - 29871.0f) < 0.6f && fabsf(r.alt_m - 29703.0f) < 0.6f, "altitudes to 1 m");
  CHECK(fabsf(r.vz_mps - 5.37f) < 0.006f, "vertical speed to 1 cm/s");
  CHECK(r.out_temp_c == -62.0f && r.board_temp_c == -11.0f, "temperatures to 1 degree");
  CHECK(fabsf(r.vbat_v - 6.12f) < 0.021f, "battery voltage to 40 mV");

  /* 3. southern and western hemisphere, descent, below the launch point */
  t.lat_e7 = -337000000; t.lon_e7 = -1511234567; t.vz_mps = -38.2f; t.alt_m = -42.0f; t.state = 4;
  TelemetryEncode(&t, b);
  CHECK(TelemetryDecode(b, TP_SIZE, &r) == 0 && r.lat_e7 == t.lat_e7 && r.lon_e7 == t.lon_e7,
        "negative latitude and longitude");
  CHECK(fabsf(r.vz_mps + 38.2f) < 0.006f && r.alt_m == -42.0f && r.state == 4, "descent, negative altitude");

  /* 4. values outside the range are clamped, not wrapped; NaN does not crash */
  t.gps_alt_m = 90000.0f; t.alt_m = -5000.0f; t.vz_mps = 900.0f; t.out_temp_c = -200.0f;
  t.vbat_v = 20.0f; t.uptime_s = 100000; t.board_temp_c = NAN;
  TelemetryEncode(&t, b);
  CHECK(TelemetryDecode(b, TP_SIZE, &r) == 0, "decode of a clamped packet");
  CHECK(r.gps_alt_m == 65535.0f && r.alt_m == -1000.0f, "altitude clamps");
  CHECK(fabsf(r.vz_mps - 327.67f) < 0.001f && r.out_temp_c == -128.0f, "speed and temperature clamps");
  CHECK(fabsf(r.vbat_v - 10.2f) < 0.001f && r.uptime_s == 65535, "voltage and uptime clamps");
  CHECK(r.board_temp_c == -128.0f, "NaN becomes the lowest value");

  /* 5. damage is noticed: every single-bit error in the packet must fail the check */
  t.gps_alt_m = 1000.0f; t.alt_m = 900.0f; t.vz_mps = 5.0f; t.out_temp_c = 20.0f; t.vbat_v = 6.0f;
  t.board_temp_c = 25.0f; t.uptime_s = 60;
  TelemetryEncode(&t, b);
  int missed = 0;
  for (unsigned bit = 0; bit < TP_SIZE * 8U; bit++)
  {
    uint8_t c[TP_SIZE];
    memcpy(c, b, TP_SIZE);
    c[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));
    if (TelemetryDecode(c, TP_SIZE, &r) == 0)
    {
      missed++;
    }
  }
  CHECK(missed == 0, "all 224 single-bit errors detected");
  CHECK(TelemetryDecode(b, TP_SIZE - 1U, &r) == -1, "short packet rejected");
  b[0] = 0x55;
  CHECK(TelemetryDecode(b, TP_SIZE, &r) == -2, "foreign packet (wrong first byte) rejected");

  printf(fails ? "%d check(s) FAILED\n" : "telemetry packet: all checks passed (28 bytes)\n", fails);
  return fails != 0;
}
