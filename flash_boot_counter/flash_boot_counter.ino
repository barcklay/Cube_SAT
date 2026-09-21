
#include <SPI.h>
const int CS = D10;

void cmd(byte c) { digitalWrite(CS, LOW); SPI.transfer(c); digitalWrite(CS, HIGH); }

void waitBusy() {
  digitalWrite(CS, LOW); SPI.transfer(0x05);
  while (SPI.transfer(0) & 0x01);          // ждём, пока BUSY = 0
  digitalWrite(CS, HIGH);
}

void addr(uint32_t a) { SPI.transfer(a >> 16); SPI.transfer(a >> 8); SPI.transfer(a); }

void readData(uint32_t a, byte* buf, int n) {
  digitalWrite(CS, LOW); SPI.transfer(0x03); addr(a);
  for (int i = 0; i < n; i++) buf[i] = SPI.transfer(0);
  digitalWrite(CS, HIGH);
}

void eraseSector(uint32_t a) {
  cmd(0x06);
  digitalWrite(CS, LOW); SPI.transfer(0x20); addr(a); digitalWrite(CS, HIGH);
  waitBusy();
}

void writeData(uint32_t a, byte* buf, int n) {
  cmd(0x06);
  digitalWrite(CS, LOW); SPI.transfer(0x02); addr(a);
  for (int i = 0; i < n; i++) SPI.transfer(buf[i]);
  digitalWrite(CS, HIGH);
  waitBusy();
}

void setup() {
  Serial.begin(115200); delay(2000);
  pinMode(CS, OUTPUT); digitalWrite(CS, HIGH);
  SPI.begin();

  byte b[1];
  readData(0, b, 1);
  Serial.print("Прочитано из flash: "); Serial.println(b[0]);

  byte boots = (b[0] == 0xFF) ? 1 : b[0] + 1;  // FF = пусто, первый запуск
  eraseSector(0);
  writeData(0, &boots, 1);

  readData(0, b, 1);
  Serial.print("Записано и проверено: "); Serial.println(b[0]);
  Serial.println(b[0] == boots ? "OK" : "ОШИБКА");
}

void loop() {}

