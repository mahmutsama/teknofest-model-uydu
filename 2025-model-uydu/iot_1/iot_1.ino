#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Wire.h"
#include "LoRa_E22.h"


unsigned long lastTime = 0;
float startAltitude = 0;

float iot_sicaklik1;
Adafruit_BME280 bme;  // I2C
LoRa_E22 E22(&Serial8);

struct Tasiyici_Sinyal {
  char tip[2]="V";
  byte basinc2[4];
  byte yukseklik2[4];
  char filtreleme[5]="mess";  // 4 char + '\0'
  char komut[2] ="O";       // 1 char + '\0'
} iot1_Data;

void setup() {
Serial.begin(9600);
  Wire.begin();

  E22.begin();

if (!bme.begin()) {
    Serial.println(F("BMP280 sensörü bulunamadı!"));
    while (true) yield();
  }


  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  // Adafruit_BMP280::SAMPLING_X2,
                  // Adafruit_BMP280::SAMPLING_X16,
                  // Adafruit_BMP280::FILTER_X16,
                  // Adafruit_BMP280::STANDBY_MS_500);


  delay(1000);
}

void loop() {
  unsigned long now = millis();
    if (now - lastTime >= 1013) {

lastTime = now;


  iot_sicaklik1=bme.readTemperature();


  memcpy(iot1_Data.basinc2, &iot_sicaklik1, sizeof(float));
  Serial.println(iot_sicaklik1);

ResponseStatus rs = E22.sendFixedMessage(0, 4, 60, &iot1_Data, sizeof(Tasiyici_Sinyal));

    }
}
