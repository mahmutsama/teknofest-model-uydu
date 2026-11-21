#include "LoRa_E22.h"

float sicaklik1;
float basinc1;
uint16_t paket_sayisi;
float yukseklik1;
float yaw;
float pitch;
float roll;
float hiz;
int sayac=0;
float yukseklik2;
float basinc2;
float irtifa_farki;
char filtreleme[5];
char zamanDizisi[20];
 char hatakod[7];
float GPS_ALTITUDE=0;
char GPS_LONGITUDE[10];
   char GPS_LATITUDE[10];
float iot_1;
float iot_2;
float pil_gerilim;
int8_t uydustatu;
LoRa_E22 E22(&Serial1);

// For serialEvent()
char receivedData[5];    // 4 char + '\0'
char komut1[2];          // 1 char + '\0'
int  indexx = 0;
bool komutTamamlandi = false;

// Gönderilecek yapı
struct Yer_Sinyal {
  char tip[2] = "Y";
   byte basinc2[4];
 byte yukseklik2[4];
  char filtreleme[5] ="RHRH";  // 4 char + '\0'
  char komut[2] ="O";       // 1 char + '\0'
} Yer_Data;

struct Signal {
  char type[7];
  char filtreleme[5];
  char zamanDizisi[20];
  char hatakod[7];
   char GPS_LONGITUDE[10];
   char GPS_LATITUDE[10];
  byte temp[4];
  byte basinc[4];
  byte paket_no[4];
  byte yukseklik[4];
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


} data;

void setup() {
  Serial.begin(9600);
  E22.begin();
  delay(500);
}

void loop() {
  // 2) LoRa'dan gelen veri var mı?
  while (E22.available() > 0) {
    ResponseStructContainer rsc = E22.receiveMessage(sizeof(Signal));
    data = *(Signal*) rsc.data;

    memcpy(&sicaklik1,  data.temp,      sizeof(float));
    memcpy(&basinc1,    data.basinc,    sizeof(float));
    memcpy(&paket_sayisi,data.paket_no, sizeof(uint16_t));
    memcpy(&yukseklik1, data.yukseklik, sizeof(float));
    memcpy(&yaw,        data.yaw,       sizeof(float));
    memcpy(&pitch,      data.pitch,     sizeof(float));
    memcpy(&roll,       data.roll,      sizeof(float));
    memcpy(&hiz,        data.vy,        sizeof(float));
    memcpy(&yukseklik2, data.yukseklik2,  sizeof(float));
    memcpy(&basinc2,    data.basinc2,    sizeof(float));
    memcpy(&irtifa_farki,data.irtifa_farki, sizeof(float));
    memcpy(&GPS_ALTITUDE, data.GPS_ALTITUDE,  sizeof(float));

    memcpy(&iot_2,    data.iot_2,    sizeof(float));
    memcpy(&iot_1,data.iot_1, sizeof(float));
    memcpy(&pil_gerilim,data.pil_gerilim, sizeof(float));
     memcpy(&uydustatu,data.uydustatu, sizeof(int8_t));
    strcpy(filtreleme, data.filtreleme);
     strcpy(zamanDizisi, data.zamanDizisi);
   strcpy(GPS_LATITUDE, data.GPS_LATITUDE);
   strcpy(GPS_LONGITUDE, data.GPS_LONGITUDE);
   strcpy(hatakod, data.hatakod);


Serial.print("<");
Serial.print(paket_sayisi);
Serial.print("*");
Serial.print(uydustatu);
Serial.print("*");
Serial.print(hatakod);
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
    Serial.print(hiz);
      Serial.print("*");
       Serial.print(sicaklik1);
      Serial.print("*");
  Serial.print(pil_gerilim);
Serial.print("*");
Serial.print(GPS_LATITUDE);
     Serial.print("*");
 Serial.print(GPS_LONGITUDE);
    Serial.print("*");
     Serial.print(GPS_ALTITUDE);
    Serial.print("*");
      Serial.print(pitch);
      Serial.print("*");
      Serial.print(roll);
      Serial.print("*");
      Serial.print(yaw);
      Serial.print("*");
      Serial.print(filtreleme);
     Serial.print("*");
   Serial.print(iot_1);
    Serial.print("*");
   Serial.print(iot_2);
      Serial.print("*");
    Serial.print(data.type);
        Serial.println(">");

    rsc.close();
    if (sayac==1 || sayac==2 || sayac==3 ) {
      if(sayac==3){
  Yer_Data.komut[0]='B';
    }
    ResponseStatus rs = E22.sendFixedMessage(0, 2, 65, &Yer_Data, sizeof(Yer_Sinyal));

  sayac++;
  Serial.println(Yer_Data.komut);
    }



  }

}

// --------------------------------------------------
// Bu fonksiyonu, loop() biter bitmez Arduino otomatik çağırır.
// --------------------------------------------------
void serialEvent() {
  while (Serial.available()) {
    char incomingByte = Serial.read();

    // 1) Tek karakterli komut mu?
    if (incomingByte == 'A' || incomingByte == 'K' || incomingByte == 'L') {
      komut1[0] = incomingByte;
      komut1[1] = '\0';
      strcpy(Yer_Data.komut, komut1);

      indexx = 0;
      komutTamamlandi = true;

    }
    // 2) Filtreleme kelimesi karakterleri mi?
    else if (!komutTamamlandi && incomingByte != '\n' && incomingByte != '\r') {
      receivedData[indexx++] = incomingByte;

      // 4 karakter tamamlandı mı?
      if (indexx == 4) {
        receivedData[indexx] = '\0';
        strcpy(Yer_Data.filtreleme, receivedData);

        indexx = 0;
        komutTamamlandi = true;
      }
    }
  }

  // Eğer komut veya filtreleme tamamlandıysa, bir kere LoRa ile gönder
  if (komutTamamlandi) {
    sayac=1;
  // ResponseStatus rs = E22.sendFixedMessage(0, 2, 65, &Yer_Data, sizeof(Yer_Sinyal));
    // Serial.print("Gönderilen komut: "); Serial.println(Yer_Data.komut);
    // Serial.print("Gönderilen filtre: "); Serial.println(Yer_Data.filtreleme);
  }

  // Bir sonraki seri veriye hazır ol
  komutTamamlandi = false;
}
