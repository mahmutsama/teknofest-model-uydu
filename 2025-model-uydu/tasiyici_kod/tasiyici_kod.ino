#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Wire.h"
#include "LoRa_E22.h"


unsigned long lastTime = 0;
int buzzer_pin=25;
//float startAltitude = 0;
float basinc2;
float yukseklik2;
float startAltitude = 0;
Adafruit_BME280 bme;  // I2C
LoRa_E22 E22(&Serial3);

struct Tasiyici_Sinyal {
  char tip[2]="T";
  byte basinc2[4];
 byte yukseklik2[4];
  char filtreleme[5]="mess";  // 4 char + '\0'
  char komut[2] ="O";       // 1 char + '\0'
} Tasiyici_Data;

void setup() {
Serial.begin(9600);
  Wire.begin();

  E22.begin();
    pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, LOW);



if (!bme.begin()) {
    Serial.println(F("BMP280 sensörü bulunamadı!"));
    while (true) yield();
  }

startAltitude = bme.readAltitude(1013.25);

  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  // Adafruit_BMP280::SAMPLING_X2,
                  // Adafruit_BMP280::SAMPLING_X16,
                  // Adafruit_BMP280::FILTER_X16,
                  // Adafruit_BMP280::STANDBY_MS_500);

  // startAltitude = bme.readAltitude(1013.25);
  delay(1000);
}

void loop() {
  unsigned long now = millis();
    if (now - lastTime >= 1000) {

lastTime = now;

   basinc2 = bme.readPressure()/100;
  float currentAltitude = bme.readAltitude(1013.25); // Anlık mutlak irtifayı oku
 float gorevIrtifasi = currentAltitude - startAltitude;
    if (gorevIrtifasi < 0) { // Küçük sapmalar için
      gorevIrtifasi = 0;
    }
    if (gorevIrtifasi > 100.0) {
      digitalWrite(buzzer_pin, HIGH);
      Serial.println(">>> 100m GECILDI! BUZZER AKTIF! <<<");
    }
  //  yukseklik2 = bme.readAltitude(1013.25) - startAltitude;
  // if (yukseklik2 < 0) yukseklik2 = 0;


  memcpy(Tasiyici_Data.basinc2, &basinc2, sizeof(float));
 // memcpy(Tasiyici_Data.yukseklik2, &yukseklik2, sizeof(float));

ResponseStatus rs = E22.sendFixedMessage(0, 2, 65, &Tasiyici_Data, sizeof(Tasiyici_Sinyal));

Serial.println(basinc2);
    }
}
