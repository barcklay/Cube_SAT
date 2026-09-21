#include <SPI.h>

const int CS_PIN = D10;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();

  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x9F);
  byte maker = SPI.transfer(0);
  byte type = SPI.transfer(0);
  byte size = SPI.transfer(0);
  digitalWrite(CS_PIN, HIGH);

  Serial.print("ID: ");
  Serial.print(maker, HEX); Serial.print(" ");
  Serial.print(type, HEX); Serial.print(" ");
  Serial.println(size, HEX);
}

void loop() {
}
