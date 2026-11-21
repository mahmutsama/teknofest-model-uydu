////////////////////////////////////////////////////////////////////////////////
//          BİRLEŞTİRİLMİŞ VE BME280 SENSÖRÜNE GÖRE GÜNCELLENMİŞ KOD          //
//   PCB GUNCELLEMESI: LoRa -> Serial8 | GPS -> Serial7 | Buzzer -> 25        //
//                     Lora_iot (Serial4) kaldirildi | servolar ESKI duzen    //
////////////////////////////////////////////////////////////////////////////////

#include "LoRa_E22.h"
#include <Adafruit_BME280.h>  // DEĞİŞTİRİLDİ: Tekrar BME280'e dönüldü
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
char fileName[30];  // Her çalıştırmada dosya adını saklamak için global değişken
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
int buzzer_pin = 25;   // DEĞİŞTİRİLDİ: 1 -> 25 (PCB'ye gore)
int kamera_pin = 13;
int kamera_buton_bas = 0;
int kamera_buton_cek = 0;
int statu_sayac;

float lat, lng;
float yaw, pitch, roll;
float GPS_ALTITUDE = 0;
char komut[2];
int32_t paket_sayisi = 1;
float sicaklik1;
float basinc1;
float yukseklik1 = 0;
int32_t uydustatu;
char zamanDizisi[20];
float startAltitude = 0;

unsigned long last_time = 0;
unsigned long tasiyici_zaman = 0;
unsigned long kamera_kayit = 0;
unsigned long kontrol_sure = 0;
unsigned long hiz_zaman = 0;
//////////////////////////////////////////////////// HIZ HESAPLAMA DEĞİŞKENLERİ
float anlik_dikey_hiz = 0.0;    // YENİ: Hızı saklamak için yeni değişkenimiz
float onceki_yukseklik1 = 0.0;  // YENİ: Bir önceki irtifayı saklamak için
float hiz_gonderilecek;

//////////////////////////////////////////////////////// FİLTRELEME (SERVO) DEĞİŞKENLERİ
char filtreleme[5] = "RHRH";

int filtreSayac = 0;
int fonksiyon1 = 0;
unsigned long ilkSure = 0;
PWMServo myservo1;
PWMServo myservo2;
PWMServo myservo3;
const int servoPos = 50;

//////////////////////////////////////////////////////// SENSÖR VE HABERLEŞME NESNELERİ
Adafruit_BME280 bme;  // DEĞİŞTİRİLDİ: Tekrar BME280'e dönüldü
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

LoRa_E22 E22(&Serial8);   // DEĞİŞTİRİLDİ: Serial2 -> Serial8 (PCB'ye gore)
// Lora_iot KALDIRILDI (tek LoRa kaldi)

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

  byte GPS_ALTITUDE[4];

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

// struct Yer_Sinyal {
//   char tip[2];
//   byte basinc2[4];
//   byte yukseklik2[4];
//   char filtreleme[5];
//   char komut[2];
// };

//////////////////////////////////////////////////////// SETUP
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial7.begin(9600);   // DEĞİŞTİRİLDİ: GPS artik Serial7 (eski Serial8.begin)

  E22.begin();           // LoRa Serial8 uzerinden (yukarida tanimli)
  // Lora_iot.begin();   // KALDIRILDI

  setSyncProvider(getTeensy3Time);
  analogReadResolution(12);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(kamera_pin, OUTPUT);
  myservo2.attach(3);    // ESKI DUZEN korundu
  myservo1.attach(4);    // ESKI DUZEN korundu
  myservo3.attach(2);    // ESKI DUZEN korundu
  myservo3.write(120);
  myservo2.write(servoPos);
  myservo1.write(servoPos);

  if (!bme.begin()) {  // DEĞİŞTİRİLDİ: BME280 başlatılıyor
    Serial.println(F("BME280 sensörü bulunamadı!"));
    while (true) yield();
  }

  if (!bno.begin()) {
    Serial.println("BNO055 sensörü bulunamadı!");
    while (true) yield();
  }

  bno.setExtCrystalUse(true);
  //////////////////////////// SD KART BAŞLATMA VE YENİ DOSYA OLUŞTURMA MANTIĞI
  if (!SD.begin(chipSelect)) {
    Serial.println("Kart basarisiz oldu veya takili degil!");
  } else {
    Serial.println("kart baslatildi.");

    int fileIndex = 0;
    while (true) {
      // Dosya adını oluştur: datalog0.csv, datalog1.csv...
      sprintf(fileName, "datalog%d.csv", fileIndex);
      // Bu isimde bir dosya var mı diye kontrol et
      if (!SD.exists(fileName)) {
        // Dosya yoksa, bu ismi kullanmak için döngüden çık
        break;
      }
      // Dosya varsa, bir sonraki numarayı denemek için indeksi artır
      fileIndex++;
    }

    // Yeni dosyayı oluştur ve başlığı yaz
    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("PaketNo,UyduStatusu,HataKodu,GondermeSaati,Basinc1,Yukseklik1,IrtifaFarki,inisHizi,Sicaklik,Pil_gerilimi,GPS_Lat,GPS_Lon,GPS_Alt,Pitch,Roll,Yaw,RHRH,IoT1,IoT2,Takim_no");
      dataFile.close();
      Serial.println("Dosya basligi yazildi.");
    } else {
      Serial.println("Hata: Dosya olusturulamadi!");
    }
  }
  // Başlangıç değerleri ayarlanıyor
  startAltitude = bme.readAltitude(1013.25);  // DEĞİŞTİRİLDİ: bme kullanılıyor

  tepe_noktasi = bme.readAltitude(1013.25) - startAltitude;  // DEĞİŞTİRİLDİ: bme kullanılıyor
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


  if (telemetri_sonlandirma == 0) {
    if (now - last_time >= 1000) {
      last_time = now;

      sicaklik1 = bme.readTemperature();
      basinc1 = bme.readPressure() / 100.0;
      yukseklik1 = bme.readAltitude(1013.25) - startAltitude;

      if (yukseklik1 < 0) yukseklik1 = 0;
      float dt = (now - hiz_zaman) / 1000.0;

      // yükseklik farkını zamana böl → m/s
      anlik_dikey_hiz = (yukseklik1 - onceki_yukseklik1) / dt;
      anlik_dikey_hiz = fabs(anlik_dikey_hiz);
      // bir sonraki hesap için değerleri güncelle
      onceki_yukseklik1 = yukseklik1;

      hiz_gonderilecek = anlik_dikey_hiz;
      hiz_zaman = now;

      if (yukseklik1 > tepe_noktasi) {
        tepe_noktasi = yukseklik1;
      }

      imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      yaw = eul.x();
      pitch = eul.y();
      roll = eul.z();
      if (now - kontrol_sure > 3050) {
        if (now - tasiyici_zaman > 3000) {
          data2.hatakod[2] = '1';
        } else {
          data2.hatakod[2] = '0';
        }

        if (now - gps_zaman > 3000) {
          data2.hatakod[3] = '1';
        } else {
          data2.hatakod[3] = '0';
        }
      }

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
      Serial.println(tepe_noktasi);
      Serial.println(uydustatu);
      Serial.println(uydustatu);
      Serial.println(yaw);

      if (uydustatu == 3) {
        myservo3.write(70);
      }
      if (uydustatu == 5) {
        if (kurtarma_baslangic == 0) {
          // 5’e İLK kez girildiği an
          kurtarma_baslangic = millis();
          kayit_zaman = millis();
          digitalWrite(buzzer_pin, HIGH);
          digitalWrite(kamera_pin, HIGH);
        } else {
          // 10 saniye geçti mi? (overflow güvenli: hep çıkarma ile kontrol et)
          if (millis() - kayit_zaman > 100) {
            digitalWrite(kamera_pin, LOW);
          }
          if ((millis() - kurtarma_baslangic >= 10000)) {
            // === 10 sn sonra yapılacak işlemler ===
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
      memcpy(data2.GPS_ALTITUDE, &GPS_ALTITUDE, sizeof(float));
      memcpy(data2.uydustatu, &uydustatu, sizeof(int32_t));
      memcpy(data2.pil_gerilim, &batteryVoltage, sizeof(float));

      memcpy(data2.vy, &hiz_gonderilecek, sizeof(float));
      strcpy(data2.zamanDizisi, zamanDizisi);

      ResponseStatus rs = E22.sendFixedMessage(0, 3, 65, &data2, sizeof(Signal));
      // Serial.println(rs.getResponseDescription()); // Debug için açılabilir
      paket_sayisi++;
      Serial.println(pitch);
      logDataToSD(batteryVoltage);
    }
  }

  while (Serial7.available() > 0) {          // DEĞİŞTİRİLDİ: Serial8 -> Serial7 (GPS)

    gps.encode(Serial7.read());              // DEĞİŞTİRİLDİ: Serial8 -> Serial7 (GPS)
    if (gps.location.isUpdated()) {
      gps_zaman = millis();
      lat = gps.location.lat();
      lng = gps.location.lng();
      sprintf(data2.GPS_LATITUDE, "%.6f", lat);
      sprintf(data2.GPS_LONGITUDE, "%.6f", lng);
    }
    if (gps.altitude.isValid()) {
      GPS_ALTITUDE = gps.altitude.meters();
    }
  }



  while (E22.available() > 1) {
    ResponseStructContainer rsc = E22.receiveMessage(sizeof(Tasiyici_Sinyal));
    char tip[2];
    memcpy(tip, rsc.data, 2);

    if (strcmp(tip, "Y") == 0) {
      struct Tasiyici_Sinyal Yer_Data = *(Tasiyici_Sinyal*)rsc.data;
      if (strcmp(data2.filtreleme, Yer_Data.filtreleme) != 0) {
        fonksiyon1 = 1;
        strcpy(filtreleme, Yer_Data.filtreleme);
        strcpy(data2.filtreleme, Yer_Data.filtreleme);
      } else if (strcmp(komut, Yer_Data.komut) != 0) {
        strcpy(komut, Yer_Data.komut);
        if (strcmp(komut, "K") == 0) {
          startAltitude = bme.readAltitude(1013.25);  // DEĞİŞTİRİLDİ: bme kullanılıyor

        } else if (strcmp(komut, "A") == 0) {
          myservo3.write(120);

          Serial.println("Ayrılma tetiklendi");
        } else if (strcmp(komut, "L") == 0) {
          myservo3.write(70);
          data2.hatakod[4] = '0';
        }
        else if (strcmp(komut, "P") == 0) {
          myservo2.write(70);

        }
        else if (strcmp(komut, "M") == 0) {
          myservo2.write(120);

        }
        else if (strcmp(komut, "J") == 0) {
          myservo1.write(70);

        }
        else if (strcmp(komut, "V") == 0) {
          myservo1.write(120);

        }
        else if (strcmp(komut, "T") == 0) {


        }
        else if (strcmp(komut, "B") == 0) {

        }
      }
    }
    rsc.close();
  }
}
//////////////////////////////////////
float customPressureToAltitude(float atmosphericPressure) {
  const float seaLevelPressure = 1013.25;
  return 44330.0 * (1.0 - pow((atmosphericPressure / seaLevelPressure), 0.1903));
}

/////////////////////////// SAAT FONKSİYONU
time_t getTeensy3Time() {
  return Teensy3Clock.get();
}
void logDataToSD(float batteryVoltage) {
  // // `setup` içinde belirlenen `fileName` kullanılıyor
  // File dataFile = SD.open(fileName, FILE_WRITE);
  // if (dataFile) {
  //   String dataString = "";
  //   dataString += String(paket_sayisi);
  //   dataString += ",";
  //   dataString += String(uydustatu);
  //   dataString += ",";
  //   dataString += String(data2.hatakod);
  //   dataString += ",";
  //   dataString += String(zamanDizisi);
  //   dataString += ",";
  //   dataString += String(basinc1);
  //   dataString += ",";
  //   dataString += String(yukseklik1);
  //   dataString += ",";
  //   dataString += String( hiz_gonderilecek);
  //   dataString += ",";
  //   dataString += String(sicaklik1);
  //   dataString += ",";
  //   dataString += String(batteryVoltage);
  //   dataString += ",";
  //   dataString += String(data2.GPS_LATITUDE);
  //   dataString += ",";
  //   dataString += String(data2.GPS_LONGITUDE);
  //   dataString += ",";
  //   dataString += String(GPS_ALTITUDE);
  //   dataString += ",";
  //   dataString += String(pitch);
  //   dataString += ",";
  //   dataString += String(roll);
  //   dataString += ",";
  //   dataString += String(yaw);
  //   dataString += ",";
  //   dataString += String(data2.filtreleme);
  //   dataString += ",";
  //   dataString += String(data2.type);

  //   dataFile.println(dataString);
  //   dataFile.close();
  // } else {
  //   // Bu hata genellikle sadece loop sırasında kart çıkarılırsa görülür
  //   Serial.print("Hata: ");
  //   Serial.print(fileName);
  //   Serial.println(" dosyasi acilamadi!");
  // }
}
