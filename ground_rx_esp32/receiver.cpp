/* HAB-1 ground receiver on an Arduino Nano ESP32 (HW-29).
 *
 * Listens for the telemetry packets of the payload and prints every packet as one text
 * line over USB, in the same format as the NUCLEO receiver build (cmake preset "Ground"),
 * so tools/ground_station.py works with either. NOTHING IS EVER TRANSMITTED.
 *
 *   RX t=<ms> len=<n> rssi=<dBm> snr=<dB> data=<hex>     a packet
 *   RXBAD t=<ms> rssi=<dBm> snr=<dB>                     a packet damaged on the air
 *   GS: alive t=<ms> rx=<n> bad=<n> noise=<dBm>          every 5 s
 *
 * The radio is driven by the RadioLib library, not by our own driver from the payload
 * firmware (hab_bringup/Core/Inc/lora_e22.h). Two independent implementations: if this
 * side hears the payload, our own driver sets the radio up correctly.
 *
 * Wiring, E22-400M22S -> Nano ESP32 (3.3 V logic, no level shifter needed):
 *   VCC  -> 3V3        GND  -> GND
 *   NSS  -> D9         MOSI -> D11        MISO -> D12        SCK -> D13
 *   NRST -> D10        BUSY -> D8         DIO1 -> D7
 * (NSS and NRST are the other way round than first planned: that is how the wires ended
 *  up on the bench, found by autoDetect() below; any pin can be NSS on an ESP32.)
 *   RXEN -> D6         TXEN -> D5
 *
 * Build and upload:
 *   arduino-cli compile --fqbn arduino:esp32:nano_nora ground_rx_esp32
 *   arduino-cli upload  --fqbn arduino:esp32:nano_nora -p /dev/cu.usbmodemXXXX ground_rx_esp32
 */
#include <Arduino.h>
#include <RadioLib.h>

/* Must match hab_bringup/Core/Inc/lora_e22.h */
const float FREQ_MHZ = 434.5;
const float BW_KHZ = 125.0;
const uint8_t SF = 9;
const uint8_t CR = 5;            /* coding rate 4/5 */
const uint16_t PREAMBLE = 8;
const float TCXO_V = 1.8;        /* the module's 32 MHz TCXO is fed from DIO3 */

SX1268 radio = new Module(D9 /* NSS */, D7 /* DIO1 */, D10 /* NRST */, D8 /* BUSY */);

volatile bool packet_flag = false;
uint32_t rx_count = 0, bad_count = 0, alive_ms = 0;
bool radio_ok = false;

void ARDUINO_ISR_ATTR onPacket() {
  packet_flag = true;
}

bool radioStart() {
  /* power -9 dBm is only a parameter of the setup call: this sketch never transmits */
  int16_t rc = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, -9, PREAMBLE, TCXO_V, true);
  if (rc != RADIOLIB_ERR_NONE) {
    Serial.printf("GS: radio not ready (RadioLib code %d), next try in 1 s\r\n", rc);
    return false;
  }
  radio.setCRC(true);
  radio.setRfSwitchPins(D6 /* RXEN */, D5 /* TXEN */);
  radio.setPacketReceivedAction(onPacket);
  rc = radio.startReceive();
  if (rc != RADIOLIB_ERR_NONE) {
    Serial.printf("GS: cannot start receiving (RadioLib code %d)\r\n", rc);
    return false;
  }
  Serial.printf("GS: listening on %lu Hz SF%u BW125\r\n", (unsigned long)(FREQ_MHZ * 1e6), SF);
  return true;
}

/* ---- Wiring check, printed at power-on. Slow bit-banged SPI, no library: it shows what
   every wire does, so a swapped or loose wire can be named. Nothing is transmitted. ---- */
static const uint8_t P_NSS = D9, P_MOSI = D11, P_MISO = D12, P_SCK = D13, P_NRST = D10, P_BUSY = D8,
                     P_DIO1 = D7, P_RXEN = D6, P_TXEN = D5;

static const char *lineState(uint8_t pin) {
  pinMode(pin, INPUT_PULLDOWN);
  delay(2);
  int down = digitalRead(pin);
  pinMode(pin, INPUT_PULLUP);
  delay(2);
  int up = digitalRead(pin);
  pinMode(pin, INPUT);
  return down == 0 && up == 1 ? "free (nothing drives it)" : down == 0 ? "held LOW" : up == 1 ? "held HIGH" : "?";
}

static uint8_t bbByte(uint8_t out) {
  uint8_t in = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(P_MOSI, (out >> i) & 1);
    delayMicroseconds(20);
    digitalWrite(P_SCK, HIGH);
    delayMicroseconds(20);
    in = (uint8_t)((in << 1) | digitalRead(P_MISO));
    digitalWrite(P_SCK, LOW);
  }
  return in;
}

static bool wiringCheck() {
  static const struct { uint8_t pin; const char *name; } all[] = {
      {D2, "D2  (unused)"}, {D3, "D3  (unused)"}, {D4, "D4  (unused)"}, {D5, "D5  TXEN"}, {D6, "D6  RXEN"},
      {D7, "D7  DIO1"}, {D8, "D8  BUSY"}, {D9, "D9  NSS"}, {D10, "D10 NRST"}, {D11, "D11 MOSI"},
      {D12, "D12 MISO"}, {D13, "D13 SCK"}};
  Serial.print("DIAG: every pin left alone, module powered (BUSY and DIO1 should be held LOW):\r\n");
  for (auto &p : all) {
    Serial.printf("DIAG:   %-13s %s\r\n", p.name, lineState(p.pin));
  }
  pinMode(P_NSS, OUTPUT);
  digitalWrite(P_NSS, HIGH);
  pinMode(P_SCK, OUTPUT);
  digitalWrite(P_SCK, LOW);
  pinMode(P_MOSI, OUTPUT);
  pinMode(P_MISO, INPUT_PULLUP);
  pinMode(P_BUSY, INPUT_PULLUP);
  pinMode(P_NRST, OUTPUT);
  digitalWrite(P_NRST, LOW);
  delay(3);
  int busy_in_reset = digitalRead(P_BUSY);
  digitalWrite(P_NRST, HIGH);
  int busy_after = 1;
  uint32_t t0 = millis();
  while (millis() - t0 < 50 && (busy_after = digitalRead(P_BUSY)) == 1) {
  }
  Serial.printf("DIAG: reset: BUSY=%d while NRST is low, BUSY=%d after release (a good module: 1 then 0)\r\n",
                busy_in_reset, busy_after);
  digitalWrite(P_NSS, LOW);
  delayMicroseconds(200);
  bbByte(0xC0);
  uint8_t st = bbByte(0x00);                 /* GetStatus */
  digitalWrite(P_NSS, HIGH);
  delay(1);
  digitalWrite(P_NSS, LOW);
  delayMicroseconds(200);
  bbByte(0x1D); bbByte(0x07); bbByte(0x40); bbByte(0x00);   /* ReadRegister 0x0740 */
  uint8_t r0 = bbByte(0x00), r1 = bbByte(0x00);
  digitalWrite(P_NSS, HIGH);
  pinMode(P_MISO, INPUT_PULLDOWN);
  delay(2);
  int miso_idle_down = digitalRead(P_MISO);
  Serial.printf("DIAG: status 0x%02X (0x00 or 0xFF = no answer), sync word %02X %02X (expected 14 24), "
                "MISO idle with pull-down %d\r\n", st, r0, r1, miso_idle_down);
  for (uint8_t pin : {P_NSS, P_SCK, P_MOSI, P_MISO, P_BUSY, P_NRST}) {
    pinMode(pin, INPUT);
  }
  return r0 == 0x14 && r1 == 0x24;
}

/* ---- Find the wires by themselves. Used only while the wiring is being sorted out. ----
   Step 1: pull one pin low at a time and watch the others. The pin that makes BUSY jump
   is NRST; the pin that makes another free pin become driven is NSS, and that other pin
   is MISO (the module drives MISO only while NSS is low).
   Step 2: with NSS and MISO known, try every pair of the remaining pins as MOSI and SCK
   until the sync word register reads 14 24. */
static const uint8_t CAND[] = {D2, D3, D4, D5, D6, D9, D10, D11, D12, D13};
static const char *CAND_NAME[] = {"D2", "D3", "D4", "D5", "D6", "D9", "D10", "D11", "D12", "D13"};
static const int NCAND = sizeof(CAND);

static int levelPair(uint8_t pin) {          /* 0 = held low, 3 = held high, 1 = free */
  pinMode(pin, INPUT_PULLDOWN);
  delay(1);
  int down = digitalRead(pin);
  pinMode(pin, INPUT_PULLUP);
  delay(1);
  int up = digitalRead(pin);
  pinMode(pin, INPUT);
  return down * 2 + up;
}

static bool tryRead(uint8_t nss, uint8_t miso, uint8_t mosi, uint8_t sck) {
  pinMode(nss, OUTPUT); digitalWrite(nss, HIGH);
  pinMode(sck, OUTPUT); digitalWrite(sck, LOW);
  pinMode(mosi, OUTPUT); digitalWrite(mosi, LOW);
  pinMode(miso, INPUT_PULLUP);
  delay(1);
  digitalWrite(nss, LOW);
  delayMicroseconds(300);
  uint8_t tx[6] = {0x1D, 0x07, 0x40, 0x00, 0x00, 0x00}, rx[6];
  for (int b = 0; b < 6; b++) {
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
      digitalWrite(mosi, (tx[b] >> i) & 1);
      delayMicroseconds(20);
      digitalWrite(sck, HIGH);
      delayMicroseconds(20);
      in = (uint8_t)((in << 1) | digitalRead(miso));
      digitalWrite(sck, LOW);
    }
    rx[b] = in;
  }
  digitalWrite(nss, HIGH);
  pinMode(sck, INPUT); pinMode(mosi, INPUT); pinMode(miso, INPUT);
  return rx[4] == 0x14 && rx[5] == 0x24;
}

static void autoDetect() {
  int nrst = -1, nss = -1, miso = -1;
  Serial.print("AUTO: looking for the wires...\r\n");
  for (int x = 0; x < NCAND; x++) {
    int busy0 = digitalRead(D8), dio0 = digitalRead(D7);
    pinMode(CAND[x], OUTPUT);
    digitalWrite(CAND[x], LOW);
    delay(3);
    int busy1 = digitalRead(D8), dio1 = digitalRead(D7);
    if (busy1 != busy0 || dio1 != dio0) {
      Serial.printf("AUTO:   %s low -> BUSY(D8) %d->%d, DIO1(D7) %d->%d: this pin is NRST\r\n", CAND_NAME[x],
                    busy0, busy1, dio0, dio1);
      nrst = x;
    }
    for (int y = 0; y < NCAND; y++) {
      if (y == x) {
        continue;
      }
      int lv = levelPair(CAND[y]);
      if (lv != 1) {
        Serial.printf("AUTO:   %s low -> %s becomes held %s: NSS and MISO\r\n", CAND_NAME[x], CAND_NAME[y],
                      lv == 0 ? "LOW" : "HIGH");
        if (nss < 0) {
          nss = x;
          miso = y;
        }
      }
    }
    pinMode(CAND[x], INPUT_PULLUP);
    delay(5);
  }
  if (nss < 0) {
    Serial.print("AUTO: no pin behaves like NSS+MISO: the NSS or the MISO wire does not reach the module\r\n");
    return;
  }
  for (int a = 0; a < NCAND; a++) {
    for (int b = 0; b < NCAND; b++) {
      if (a == b || a == nss || a == miso || b == nss || b == miso || a == nrst || b == nrst) {
        continue;
      }
      if (tryRead(CAND[nss], CAND[miso], CAND[a], CAND[b])) {
        Serial.printf("AUTO: FOUND. NSS=%s MISO=%s MOSI=%s SCK=%s NRST=%s\r\n", CAND_NAME[nss], CAND_NAME[miso],
                      CAND_NAME[a], CAND_NAME[b], nrst < 0 ? "not found" : CAND_NAME[nrst]);
        return;
      }
    }
  }
  Serial.printf("AUTO: NSS=%s MISO=%s found, but no pair of pins works as MOSI+SCK: one of those two "
                "wires does not reach the module\r\n", CAND_NAME[nss], CAND_NAME[miso]);
}

void setup() {
  Serial.begin(115200);
  delay(1500);                   /* give the USB port time to appear on the laptop */
  Serial.print("\r\n=== HAB-1 ground receiver (Nano ESP32) ===\r\n");
  if (!wiringCheck()) {
    autoDetect();
  }
}

void loop() {
  if (!radio_ok) {
    radio_ok = radioStart();
    if (!radio_ok) {
      delay(1000);
      return;
    }
    alive_ms = millis();
  }

  if (packet_flag) {
    packet_flag = false;
    uint8_t pkt[64];
    size_t n = radio.getPacketLength();
    if (n > sizeof(pkt)) {
      n = sizeof(pkt);
    }
    int16_t rc = radio.readData(pkt, n);
    int rssi = (int)lroundf(radio.getRSSI());
    int snr = (int)lroundf(radio.getSNR());
    if (rc == RADIOLIB_ERR_NONE && n > 0) {
      rx_count++;
      Serial.printf("RX t=%lu len=%u rssi=%d snr=%d data=", (unsigned long)millis(), (unsigned)n, rssi, snr);
      for (size_t i = 0; i < n; i++) {
        Serial.printf("%02X", pkt[i]);
      }
      Serial.print("\r\n");
    } else if (rc == RADIOLIB_ERR_CRC_MISMATCH) {
      bad_count++;
      Serial.printf("RXBAD t=%lu rssi=%d snr=%d\r\n", (unsigned long)millis(), rssi, snr);
    }
    radio.startReceive();
  }

  if (millis() - alive_ms >= 5000U) {
    alive_ms = millis();
    Serial.printf("GS: alive t=%lu rx=%lu bad=%lu noise=%d\r\n", (unsigned long)alive_ms,
                  (unsigned long)rx_count, (unsigned long)bad_count, (int)lroundf(radio.getRSSI(false)));
  }
}
