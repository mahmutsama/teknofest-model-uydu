////////////////////////////////////////////////////////////////////////////////
//          BİRLEŞTİRİLMİŞ VE BME280 SENSÖRÜNE GÖRE GÜNCELLENMİŞ KOD          //
////////////////////////////////////////////////////////////////////////////////

#include "LoRa_E22.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Wire.h"
#include <TimeLib.h>
#include <PWMServo.h>
#include <TinyGPS++.h>
#include "uydustat.h"
// SD KART İÇİN EKLENEN KÜTÜPHANELER
#include <SD.h>
#include <SPI.h>

///////////////////////////////////SD KART İÇİN EKLENEN DEĞİŞKENLER
const int chipSelect = BUILTIN_SDCARD;
char fileName[30];

//////////////////////////////////////////////////////// HATA KODLARI
TinyGPSPlus gps;

/////////////////////////////////////////// PİL VOLTAJI DEĞİŞKENLERİ
const int batteryPin = 14;
const float R1 = 15000.0;
const float R2 = 4700.0;
const float V_REF = 3.3;

/////////////////////////////////////////// GLOBAL DEĞİŞKENLER
int telemetri_sonlandirma = 0;
float tepe_noktasi = 0;
unsigned long gps_zaman = 0;
unsigned long kurtarma_baslangic = 0;
unsigned long kayit_zaman = 0;
int buzzer_pin = 1;
int kamera_pin = 13;
int kamera_buton_bas = 0;
int kamera_buton_cek = 0;
int statu_sayac;

float lat, lng;
float yaw, pitch, roll;
float iot_1 = 0;
float iot_2 = 0;
float GPS_ALTITUDE = 0;
char komut[2];
int32_t paket_sayisi = 1;
float sicaklik1;
float basinc1, basinc2;
float yukseklik1 = 0, yukseklik2 = 0;
int32_t uydustatu;
float irtifa_farki;
char zamanDizisi[20];
float startAltitude = 0;
float startAltitude2 = 0;
unsigned long last_time = 0;
unsigned long tasiyici_zaman = 0;
unsigned long kamera_kayit = 0;
unsigned long kontrol_sure = 0;
unsigned long hiz_zaman = 0;

//////////////////////////////////////////////////// HIZ HESAPLAMA DEĞİŞKENLERİ
float anlik_dikey_hiz = 0.0;
float onceki_yukseklik1 = 0.0;
float hiz_gonderilecek;

//////////////////////////////////////////////////////// FİLTRELEME (SERVO) DEĞİŞKENLERİ
char filtreleme[5] = "RHRH";
int sure1 = 0, sure2 = 0;
char renk1 = 0, renk2 = 0;
int filtreSayac = 0;
int fonksiyon1 = 0;
unsigned long ilkSure = 0;
PWMServo myservo1;
PWMServo myservo2;
PWMServo myservo3;
const int servoPos = 50;

//////////////////////////////////////////////////////// SERİ PORT DİNLEME DEĞİŞKENLERİ
char receivedSerialData[5];
int serialCharIndex = 0;

//////////////////////////////////////////////////////// SENSÖR VE HABERLEŞME NESNELERİ
Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

LoRa_E22 E22(&Serial1);
LoRa_E22 Lora_iot(&Serial4);

//////////////////////////////////////////////////////// VERİ PAKETİ YAPILARI
struct Signal {
  char type[7] = "633113";
  char filtreleme[5] = "RHRH";
  char zamanDizisi[20];
  char hatakod[7] = "111111";
  char GPS_LONGITUDE[10] = "38.750000";
  char GPS_LATITUDE[10] = "33.330000";
  byte temp[4];
  byte basinc1[4];
  byte paket_no[4];
  byte yukseklik1[4];
  byte yaw[4];
  byte pitch[4];
  byte roll[4];
  byte vy[4];
  byte basinc2[4];
  byte yukseklik2[4];
  byte irtifa_farki[4];
  byte GPS_ALTITUDE[4];
  byte iot_1[4];
  byte iot_2[4];
  byte pil_gerilim[4];
  byte uydustatu[4];
} data2;

struct Tasiyici_Sinyal {
  char tip[2];
  byte basinc2[4];
  byte yukseklik2[4];
  char filtreleme[5];
  char komut[2] = "O";
};

//////////////////////////////////////////////////////// SETUP
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial8.begin(9600);

  E22.begin();
  Lora_iot.begin();

  setSyncProvider(getTeensy3Time);
  analogReadResolution(12);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(kamera_pin, OUTPUT);
  myservo2.attach(3);
  myservo1.attach(4);
  myservo3.attach(2);
  myservo3.write(120);
  myservo2.write(servoPos);
  myservo1.write(servoPos);

  if (!bme.begin()) {
    Serial.println(F("BME280 sensörü bulunamadı!"));
    while (true) yield();
  }

  if (!bno.begin()) {
    Serial.println("BNO055 sensörü bulunamadı!");
    while (true) yield();
  }

  bno.setExtCrystalUse(true);

  if (!SD.begin(chipSelect)) {
    Serial.println("Kart basarisiz oldu veya takili degil!");
  } else {
    Serial.println("kart baslatildi.");
    int fileIndex = 0;
    while (true) {
      sprintf(fileName, "datalog%d.csv", fileIndex);
      if (!SD.exists(fileName)) {
        break;
      }
      fileIndex++;
    }

    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("PaketNo,UyduStatusu,HataKodu,GondermeSaati,Basinc1,Basinc2,Yukseklik1,Yukseklik2,IrtifaFarki,inisHizi,Sicaklik,Pil_gerilimi,GPS_Lat,GPS_Lon,GPS_Alt,Pitch,Roll,Yaw,RHRH,IoT1,IoT2,Takim_no");
      dataFile.close();
      Serial.println("Dosya basligi yazildi.");
    } else {
      Serial.println("Hata: Dosya olusturulamadi!");
    }
  }

  startAltitude = bme.readAltitude(1013.25);
  startAltitude2 = startAltitude;
  tepe_noktasi = bme.readAltitude(1013.25) - startAltitude;
  onceki_yukseklik1 = bme.readAltitude(1013.25) - startAltitude;

  last_time = millis();
  tasiyici_zaman = millis();
  gps_zaman = millis();
  hiz_zaman = millis();
  kamera_kayit = millis();
  kontrol_sure = millis();
  delay(1000);
}

//////////////////////////////////////////////////////// LOOP
void loop() {
  unsigned long now = millis();
  if (kamera_buton_bas == 0) {
    if (now - kamera_kayit > 30000) {
      digitalWrite(kamera_pin, HIGH);
      kamera_buton_bas = 1;
    }
  }
  if (kamera_buton_cek == 0) {
    if (now - kamera_kayit > 30500) {
      digitalWrite(kamera_pin, LOW);
      kamera_buton_cek = 1;
    }
  }

  // =========================================================================
  // SERİ PORTTAN DOĞRUDAN KOMUT VE FİLTRELEME ALMA
  // =========================================================================
  while (Serial.available() > 0) {
    char inChar = Serial.read();

    if (inChar == 'A' || inChar == 'K' || inChar == 'L') {
      komut[0] = inChar;
      komut[1] = '\0';
      if (inChar == 'K') {
        startAltitude = bme.readAltitude(1013.25);
        startAltitude2 = customPressureToAltitude(basinc2);
      } else if (inChar == 'A') {
        myservo3.write(120);
        Serial.println("Ayrilma tetiklendi");
      } else if (inChar == 'L') {
        myservo3.write(70);
        data2.hatakod[4] = '0';
      }
      serialCharIndex = 0;
    }
    else if (inChar != '\n' && inChar != '\r' && inChar != ' ' && inChar != '<' && inChar != '>') {
      receivedSerialData[serialCharIndex++] = inChar;
      if (serialCharIndex == 4) {
        receivedSerialData[4] = '\0';
        strcpy(filtreleme, receivedSerialData);
        strcpy(data2.filtreleme, receivedSerialData);
        fonksiyon1 = 1;
        filtreSayac = 0;
        serialCharIndex = 0;
      }
    }
  }
  // =========================================================================

  if (telemetri_sonlandirma == 0) {
    if (now - last_time >= 1000) {
      last_time = now;

      sicaklik1 = bme.readTemperature();
      basinc1 = bme.readPressure() / 100.0;
      yukseklik1 = bme.readAltitude(1013.25) - startAltitude;

      if (yukseklik1 < 0) yukseklik1 = 0;
      float dt = (now - hiz_zaman) / 1000.0;

      anlik_dikey_hiz = (yukseklik1 - onceki_yukseklik1) / dt;
      anlik_dikey_hiz = fabs(anlik_dikey_hiz);
      onceki_yukseklik1 = yukseklik1;

      hiz_gonderilecek = anlik_dikey_hiz;
      hiz_zaman = now;

      if (yukseklik1 > tepe_noktasi) {
        tepe_noktasi = yukseklik1;
      }

      imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      yaw   = eul.x();
      pitch = eul.y();
      roll  = eul.z();

      if (now - kontrol_sure > 3050) {
        if (now - tasiyici_zaman > 3000) { data2.hatakod[2] = '1'; }
        else { data2.hatakod[2] = '0'; }

        if (now - gps_zaman > 3000) { data2.hatakod[3] = '1'; }
        else { data2.hatakod[3] = '0'; }
      }
      irtifa_farki = fabs(yukseklik2 - yukseklik1);

      if (anlik_dikey_hiz < 14 && anlik_dikey_hiz > 12) {
        data2.hatakod[0] = '0';
      } else {
        data2.hatakod[0] = '1';
      }

      if (anlik_dikey_hiz < 8 && anlik_dikey_hiz > 6) {
        data2.hatakod[1] = '0';
      } else {
        data2.hatakod[1] = '1';
      }

      uydustatu = Uydustatu_al();

      if (uydustatu == 3) {
        myservo3.write(70);
      }
      if (uydustatu == 5) {
        if (kurtarma_baslangic == 0) {
          kurtarma_baslangic = millis();
          kayit_zaman = millis();
          digitalWrite(buzzer_pin, HIGH);
          digitalWrite(kamera_pin, HIGH);
        } else {
          if (millis() - kayit_zaman > 100) {
            digitalWrite(kamera_pin, LOW);
          }
          if (millis() - kurtarma_baslangic >= 10000) {
            telemetri_sonlandirma = 1;
          }
        }
      }

      int rawValue = analogRead(batteryPin);
      float pinVoltage = rawValue * (V_REF / 4095.0);
      float batteryVoltage = pinVoltage * (R1 + R2) / R2;

      snprintf(zamanDizisi, sizeof(zamanDizisi), "%02d/%02d/%02d,%02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());

      memcpy(data2.paket_no, &paket_sayisi, sizeof(int32_t));
      memcpy(data2.temp, &sicaklik1, sizeof(float));
      memcpy(data2.basinc1, &basinc1, sizeof(float));
      memcpy(data2.yukseklik1, &yukseklik1, sizeof(float));
      memcpy(data2.yaw, &yaw, sizeof(float));
      memcpy(data2.pitch, &pitch, sizeof(float));
      memcpy(data2.roll, &roll, sizeof(float));
      memcpy(data2.irtifa_farki, &irtifa_farki, sizeof(float));
      memcpy(data2.GPS_ALTITUDE, &GPS_ALTITUDE, sizeof(float));
      memcpy(data2.uydustatu, &uydustatu, sizeof(int32_t));
      memcpy(data2.pil_gerilim, &batteryVoltage, sizeof(float));
      memcpy(data2.iot_1, &iot_1, sizeof(float));
      memcpy(data2.iot_2, &iot_2, sizeof(float));
      memcpy(data2.vy, &hiz_gonderilecek, sizeof(float));
      strcpy(data2.zamanDizisi, zamanDizisi);

      ResponseStatus rs = E22.sendFixedMessage(0, 3, 65, &data2, sizeof(Signal));

      // YER İSTASYONU SERİ PORT FORMATI
      Serial.print("<");
      Serial.print(paket_sayisi);
      Serial.print("*");
      Serial.print(uydustatu);
      Serial.print("*");
      Serial.print(data2.hatakod);
      Serial.print("*");
      Serial.print(zamanDizisi);
      Serial.print("*");
      Serial.print(basinc1);
      Serial.print("*");
      Serial.print(basinc2);
      Serial.print("*");
      Serial.print(yukseklik1);
      Serial.print("*");
      Serial.print(yukseklik2);
      Serial.print("*");
      Serial.print(irtifa_farki);
      Serial.print("*");
      Serial.print(hiz_gonderilecek);
      Serial.print("*");
      Serial.print(sicaklik1);
      Serial.print("*");
      Serial.print(batteryVoltage);
      Serial.print("*");
      Serial.print(data2.GPS_LATITUDE);
      Serial.print("*");
      Serial.print(data2.GPS_LONGITUDE);
      Serial.print("*");
      Serial.print(GPS_ALTITUDE);
      Serial.print("*");
      Serial.print(pitch);
      Serial.print("*");
      Serial.print(roll);
      Serial.print("*");
      Serial.print(yaw);
      Serial.print("*");
      Serial.print(data2.filtreleme);
      Serial.print("*");
      Serial.print(iot_1);
      Serial.print("*");
      Serial.print(iot_2);
      Serial.print("*");
      Serial.print(data2.type);
      Serial.println(">");

      paket_sayisi++;
      logDataToSD(batteryVoltage);
    }
  }

  while (Serial8.available() > 0) {
    gps.encode(Serial8.read());
    if (gps.location.isUpdated()) {
      gps_zaman = millis();
      lat = gps.location.lat();
      lng = gps.location.lng();
      sprintf(data2.GPS_LATITUDE,  "%.6f", lat);
      sprintf(data2.GPS_LONGITUDE, "%.6f", lng);
    }
    if (gps.altitude.isValid()) {
      GPS_ALTITUDE = gps.altitude.meters();
    }
  }

  while (Lora_iot.available() > 1) {
    ResponseStructContainer rsc = Lora_iot.receiveMessage(sizeof(Tasiyici_Sinyal));
    char tip[2];
    memcpy(tip, rsc.data, 2);
    if (strcmp(tip, "V") == 0) {
      struct Tasiyici_Sinyal mahmut = *(Tasiyici_Sinyal*)rsc.data;
      memcpy(&iot_1, mahmut.basinc2, sizeof(float));
    } else if (strcmp(tip, "Q") == 0) {
      struct Tasiyici_Sinyal mahmut = *(Tasiyici_Sinyal*)rsc.data;
      memcpy(&iot_2, mahmut.basinc2, sizeof(float));
    }
    rsc.close();
  }

  while (E22.available() > 1) {
    ResponseStructContainer rsc = E22.receiveMessage(sizeof(Tasiyici_Sinyal));
    char tip[2];
    memcpy(tip, rsc.data, 2);

    if (strcmp(tip, "T") == 0) {
      tasiyici_zaman = millis();
      struct Tasiyici_Sinyal mahmut = *(Tasiyici_Sinyal*)rsc.data;

      memcpy(&basinc2, mahmut.basinc2, sizeof(float));
      float mutlak_yukseklik2 = customPressureToAltitude(basinc2);

      yukseklik2 = mutlak_yukseklik2 - startAltitude2;
      if (yukseklik2 < 0) {
        yukseklik2 = 0;
      }
      memcpy(&data2.basinc2, &basinc2, sizeof(float));
      memcpy(&data2.yukseklik2, &yukseklik2, sizeof(float));
    }
    else if (strcmp(tip, "Y") == 0) {
      struct Tasiyici_Sinyal Yer_Data = *(Tasiyici_Sinyal*)rsc.data;
      if (strcmp(data2.filtreleme, Yer_Data.filtreleme) != 0) {
        fonksiyon1 = 1;
        filtreSayac = 0;
        strcpy(filtreleme, Yer_Data.filtreleme);
        strcpy(data2.filtreleme, Yer_Data.filtreleme);
      } else if (strcmp(komut, Yer_Data.komut) != 0) {
        strcpy(komut, Yer_Data.komut);
        if (strcmp(komut, "K") == 0) {
          startAltitude = bme.readAltitude(1013.25);
          startAltitude2 = customPressureToAltitude(basinc2);
        } else if (strcmp(komut, "A") == 0) {
          myservo3.write(120);
          Serial.println("Ayrilma tetiklendi");
        } else if (strcmp(komut, "L") == 0) {
          myservo3.write(70);
          data2.hatakod[4] = '0';
        }
      }
    }
    rsc.close();
  }

  // FİLTRELEME (SERVO) KONTROL MANTIĞI
  if (fonksiyon1 == 1) {
    if (filtreSayac == 0) {
      data2.hatakod[5] = '0';
      sure1 = (filtreleme[0] - '0') * 1000;
      sure2 = (filtreleme[2] - '0') * 1000;
      renk1 = filtreleme[1];
      renk2 = filtreleme[3];
      filtreSayac = 1;
      ilkSure = millis();
    } else if (filtreSayac == 1) {
      if (renk1 == 'B') { myservo2.write(15); }
      else if (renk1 == 'G') { myservo2.write(68); }
      else if (renk1 == 'R') { myservo2.write(32); }
      else if (renk1 == 'M') { myservo2.write(32); myservo1.write(29); }
      else if (renk1 == 'F') { myservo2.write(68); myservo1.write(75); }
      else if (renk1 == 'N') { myservo2.write(15); myservo1.write(7); }
      else if (renk1 == 'P') { myservo2.write(32); myservo1.write(7); }
      else if (renk1 == 'Y') { myservo2.write(32); myservo1.write(75); }
      else if (renk1 == 'C') { myservo2.write(15); myservo1.write(75); }
      filtreSayac = 2;
    } else if (filtreSayac == 2) {
      if (now - ilkSure >= sure1) {
        ilkSure = millis();
        if (renk2 == 'B') { myservo2.write(15); myservo1.write(servoPos); }
        else if (renk2 == 'G') { myservo2.write(68); myservo1.write(servoPos); }
        else if (renk2 == 'R') { myservo2.write(32); myservo1.write(servoPos); }
        else if (renk2 == 'M') { myservo2.write(32); myservo1.write(29); }
        else if (renk2 == 'F') { myservo2.write(68); myservo1.write(75); }
        else if (renk2 == 'N') { myservo2.write(15); myservo1.write(7); }
        else if (renk2 == 'P') { myservo2.write(32); myservo1.write(7); }
        else if (renk2 == 'Y') { myservo2.write(32); myservo1.write(75); }
        else if (renk2 == 'C') { myservo2.write(15); myservo1.write(75); }
        filtreSayac = 3;
      }
    } else if (filtreSayac == 3) {
      if (now - ilkSure >= sure2) {
        myservo2.write(servoPos);
        myservo1.write(servoPos);
        fonksiyon1 = 0;
        filtreSayac = 0;
      }
    }
  }
}

float customPressureToAltitude(float atmosphericPressure) {
  const float seaLevelPressure = 1013.25;
  return 44330.0 * (1.0 - pow((atmosphericPressure / seaLevelPressure), 0.1903));
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void logDataToSD(float batteryVoltage) {
  File dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    String dataString = "";
    dataString += String(paket_sayisi);
    dataString += ",";
    dataString += String(uydustatu);
    dataString += ",";
    dataString += String(data2.hatakod);
    dataString += ",";
    dataString += String(zamanDizisi);
    dataString += ",";
    dataString += String(basinc1);
    dataString += ",";
    dataString += String(basinc2);
    dataString += ",";
    dataString += String(yukseklik1);
    dataString += ",";
    dataString += String(yukseklik2);
    dataString += ",";
    dataString += String(irtifa_farki);
    dataString += ",";
    dataString += String(hiz_gonderilecek);
    dataString += ",";
    dataString += String(sicaklik1);
    dataString += ",";
    dataString += String(batteryVoltage);
    dataString += ",";
    dataString += String(data2.GPS_LATITUDE);
    dataString += ",";
    dataString += String(data2.GPS_LONGITUDE);
    dataString += ",";
    dataString += String(GPS_ALTITUDE);
    dataString += ",";
    dataString += String(pitch);
    dataString += ",";
    dataString += String(roll);
    dataString += ",";
    dataString += String(yaw);
    dataString += ",";
    dataString += String(data2.filtreleme);
    dataString += ",";
    dataString += String(iot_1);
    dataString += ",";
    dataString += String(iot_2);
    dataString += ",";
    dataString += String(data2.type);

    dataFile.println(dataString);
    dataFile.close();
  } else {
    // Serial.print("Hata: ");
    // Serial.print(fileName);
    // Serial.println(" dosyasi acilamadi!");
  }
}
