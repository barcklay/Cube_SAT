#include <Wire.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;
float p0; // давление на увроне стола = ноль высоты

void setup() {
  Serial.begin(115200);
  delay(2000);
  if (!bmp.begin_I2C(0x77)) {
    Serial.println("Sensor not found");
    while (1);
  }
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  for (int i = 0; i < 20; i++) { bmp.performReading(); delay(50); } // дать фильтру настояться
  p0 = bmp.pressure / 100.0;
} 

void loop() {
  float h = bmp.readAltitude(p0);
    Serial.print("h = "); Serial.print(h, 2); Serial.print(" m  ");
    Serial.print("P = "); Serial.print(bmp.pressure / 100.0, 2); Serial.println(" hPa");
    delay(200);
}
