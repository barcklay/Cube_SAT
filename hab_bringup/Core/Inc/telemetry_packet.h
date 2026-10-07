/* Radio telemetry packet of HAB-1 (HW-13): 28 bytes, little-endian, CRC-16 at the end.
 *
 * Pure C with no hardware in it, so the same file builds into the firmware, the ground
 * station and the test on the laptop (tools/test_telemetry_packet.c).
 *
 * Why these fields: what is needed to follow the flight and to find the payload after
 * landing if nothing else comes back. Position first, then how the flight is going, then
 * how the payload feels.
 *
 *  byte  field        meaning
 *  0     magic        0xA1: HAB-1 telemetry, format version 1
 *  1     payload_id   which payload (two fly on one trip, HW-150)
 *  2-3   seq          packet number, wraps at 65535: gaps show lost packets
 *  4-5   uptime_s     seconds since power-on (18 h range)
 *  6     state_flags  bits 0-2 flight state (PRELAUNCH..LANDED), bits 3-7 TP_FLAG_*
 *  7     gps_sats     satellites used
 *  8-11  lat_e7       degrees x 1e7, + north
 *  12-15 lon_e7       degrees x 1e7, + east
 *  16-17 gps_alt_m    GPS altitude above sea level, m (0..65535)
 *  18-19 alt_m        altitude above the launch point + 1000 m, m (so -1000..64535)
 *  20-21 vz_cms       vertical speed, cm/s, + = up (+-327 m/s)
 *  22    out_temp_c   outside temperature, deg C (-128..127)
 *  23    board_temp_c barometer chip temperature, deg C
 *  24    vbat_40mv    battery voltage in 40 mV steps (0..10.2 V), 0 = not measured
 *  25    spare        0
 *  26-27 crc          CRC-16/CCITT-FALSE over bytes 0..25
 */
#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

#include <stdint.h>

#define TP_MAGIC 0xA1U
#define TP_SIZE 28U

#define TP_FLAG_BARO_OK 0x08U   /* barometer answers */
#define TP_FLAG_GPS_FIX 0x10U   /* position in this packet is fresh */
#define TP_FLAG_OUT_OK 0x20U    /* outside thermometer answers */
#define TP_FLAG_LOG_OK 0x40U    /* the black box is recording */
#define TP_FLAG_RESUMED 0x80U   /* the board restarted in flight and carried on */

typedef struct
{
  uint8_t payload_id;
  uint16_t seq;
  uint32_t uptime_s;
  uint8_t state;       /* 0..5 */
  uint8_t flags;       /* TP_FLAG_* */
  uint8_t gps_sats;
  int32_t lat_e7;
  int32_t lon_e7;
  float gps_alt_m;
  float alt_m;         /* above the launch point */
  float vz_mps;
  float out_temp_c;
  float board_temp_c;
  float vbat_v;        /* 0 = not measured */
} Telemetry;

static inline uint16_t TpCrc16(const uint8_t *p, uint32_t n)
{
  uint16_t crc = 0xFFFFU;
  while (n--)
  {
    crc ^= (uint16_t)(*p++) << 8;
    for (int i = 0; i < 8; i++)
    {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* Round and clamp a value into [lo, hi] */
static inline int32_t TpClamp(float v, int32_t lo, int32_t hi)
{
  float r = v < 0.0f ? v - 0.5f : v + 0.5f;
  if (!(r > (float)lo))   /* also catches NaN */
  {
    return lo;
  }
  return r > (float)hi ? hi : (int32_t)r;
}

static inline void TpPut16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static inline void TpPut32(uint8_t *p, uint32_t v)
{
  TpPut16(p, (uint16_t)v);
  TpPut16(p + 2, (uint16_t)(v >> 16));
}

static inline uint16_t TpGet16(const uint8_t *p)
{
  return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t TpGet32(const uint8_t *p)
{
  return (uint32_t)TpGet16(p) | ((uint32_t)TpGet16(p + 2) << 16);
}

static inline void TelemetryEncode(const Telemetry *t, uint8_t out[TP_SIZE])
{
  out[0] = TP_MAGIC;
  out[1] = t->payload_id;
  TpPut16(out + 2, t->seq);
  TpPut16(out + 4, (uint16_t)(t->uptime_s > 65535U ? 65535U : t->uptime_s));
  out[6] = (uint8_t)((t->state & 0x07U) | (t->flags & 0xF8U));
  out[7] = t->gps_sats;
  TpPut32(out + 8, (uint32_t)t->lat_e7);
  TpPut32(out + 12, (uint32_t)t->lon_e7);
  TpPut16(out + 16, (uint16_t)TpClamp(t->gps_alt_m, 0, 65535));
  TpPut16(out + 18, (uint16_t)TpClamp(t->alt_m + 1000.0f, 0, 65535));
  TpPut16(out + 20, (uint16_t)(int16_t)TpClamp(t->vz_mps * 100.0f, -32768, 32767));
  out[22] = (uint8_t)(int8_t)TpClamp(t->out_temp_c, -128, 127);
  out[23] = (uint8_t)(int8_t)TpClamp(t->board_temp_c, -128, 127);
  out[24] = (uint8_t)TpClamp(t->vbat_v / 0.04f, 0, 255);
  out[25] = 0;
  TpPut16(out + 26, TpCrc16(out, TP_SIZE - 2U));
}

/* 0 = ok; -1 wrong length, -2 wrong magic, -3 CRC mismatch (the packet is not ours or damaged) */
static inline int TelemetryDecode(const uint8_t *in, uint32_t len, Telemetry *t)
{
  if (len != TP_SIZE)
  {
    return -1;
  }
  if (in[0] != TP_MAGIC)
  {
    return -2;
  }
  if (TpGet16(in + 26) != TpCrc16(in, TP_SIZE - 2U))
  {
    return -3;
  }
  t->payload_id = in[1];
  t->seq = TpGet16(in + 2);
  t->uptime_s = TpGet16(in + 4);
  t->state = in[6] & 0x07U;
  t->flags = in[6] & 0xF8U;
  t->gps_sats = in[7];
  t->lat_e7 = (int32_t)TpGet32(in + 8);
  t->lon_e7 = (int32_t)TpGet32(in + 12);
  t->gps_alt_m = (float)TpGet16(in + 16);
  t->alt_m = (float)TpGet16(in + 18) - 1000.0f;
  t->vz_mps = (float)(int16_t)TpGet16(in + 20) / 100.0f;
  t->out_temp_c = (float)(int8_t)in[22];
  t->board_temp_c = (float)(int8_t)in[23];
  t->vbat_v = (float)in[24] * 0.04f;
  return 0;
}

#endif /* TELEMETRY_PACKET_H */
