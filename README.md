# Model Uydu – Gömülü Uçuş Yazılımı ve Telemetri Sistemi

TEKNOFEST Model Uydu yarışması için geliştirdiğim gömülü yazılımlar ve görev yükü PCB tasarımı.
Sistem; görev yükü (uydu), taşıyıcı, yer istasyonu ve IoT istasyonlarından oluşan, LoRa ile haberleşen
çok düğümlü bir telemetri ağıdır.

*Embedded flight software and telemetry system for the TEKNOFEST Model Satellite competition
(Teensy 4.1, BME280, BNO055, GPS, LoRa E22, SD logging, servo-driven filter mechanism, custom PCB).*

---

## Sistem Mimarisi

```
                 LoRa (E22, fixed transmission)
   ┌────────────┐  basınç   ┌──────────────────┐  telemetri  ┌────────────────┐   USB/Seri   ┌──────────────┐
   │  Taşıyıcı  │ ────────▶ │    Görev Yükü    │ ──────────▶ │  Yer İstasyonu │ ───────────▶ │  Bilgisayar   │
   └────────────┘           │  (Teensy 4.1)    │ ◀────────── │                │ ◀─────────── │  (seri port)  │
   ┌────────────┐ sıcaklık  │                  │  komut /    └────────────────┘   komut      └──────────────┘
   │ IoT 1 / 2  │ ────────▶ │                  │  filtre
   └────────────┘           └──────────────────┘
```

## Donanım

| Bileşen | Kullanım |
|---|---|
| Teensy 4.1 | Ana mikrodenetleyici, dahili SD kart ve RTC |
| BME280 | Sıcaklık, basınç, barometrik irtifa |
| BNO055 | 9 eksen IMU – yaw / pitch / roll (Euler) |
| GPS (NEO, TinyGPS++) | Enlem, boylam, GPS irtifası |
| EBYTE E22 LoRa | Düğümler arası haberleşme (sabit adres/kanal) |
| 3 × Servo (PWMServo) | Ayrılma mekanizması ve renk filtreleme mekanizması |
| Buzzer, kamera tetik pini | Kurtarma sinyali ve video kaydı |
| Gerilim bölücü (15k / 4.7k) | Pil gerilimi ölçümü (12-bit ADC) |

2026 görev yükü kartının şeması ve PCB çizimi: [`2026-gorev-yuku/donanim`](2026-gorev-yuku/donanim).

## Yazılım Özellikleri

- **Uçuş durum makinesi** (`uydustat.h`): Uçuş öncesi → Yükselme → Alçalma → Ayrılma → İniş → Kurtarma.
  Geçişler irtifa ve tepe noktası farkına göre yapılır; yanlış tetiklemeyi önlemek için ardışık doğrulama sayacı kullanılır.
- **Telemetri paketi**: Paket no, uydu statüsü, hata kodu, zaman (RTC), basınç, irtifa, irtifa farkı,
  dikey hız, sıcaklık, pil gerilimi, GPS, pitch/roll/yaw, filtre komutu, IoT verileri ve takım no –
  binary `struct` olarak LoRa ile gönderilir, yer istasyonunda çözülüp `<...*...>` formatında seri porta basılır.
- **6 haneli hata kodu**: iniş hızı aralık kontrolleri (12–14 m/s ve 6–8 m/s), taşıyıcı verisi zaman aşımı,
  GPS zaman aşımı, ayrılma durumu ve filtreleme durumu.
- **Dikey hız hesabı**: ardışık barometrik irtifa farkı / geçen süre.
- **Renk filtreleme mekanizması**: Yer istasyonundan gelen 4 karakterlik komut (ör. `3R5G` → 3 sn kırmızı, 5 sn yeşil)
  ile iki servo bloklamadan (`millis()` tabanlı) konumlandırılır.
- **Uzaktan komutlar**: `K` (irtifa kalibrasyonu), `A` / `L` (ayrılma servo), 2026 sürümünde ek servo test komutları.
- **SD kart kaydı**: Her açılışta yeni `datalogN.csv` dosyası oluşturulur.
- **Kurtarma modu**: buzzer ve kamera tetiklenir, 10 sn sonra telemetri sonlandırılır.
- **Yer istasyonu**: komutların karşı tarafa ulaşması için tekrar gönderim ve paket gelmezse zaman aşımıyla garanti gönderim.

## Klasör Yapısı

```
2025-model-uydu/
├── gorev_yuku/          # Görev yükü: sensörler, durum makinesi, telemetri, filtre, SD kayıt (taşıyıcı + IoT verisiyle)
├── tasiyici_kod/        # Taşıyıcı: basınç verisini görev yüküne gönderir, 100 m üstünde buzzer
├── asil_yeristasyonu/   # Yer istasyonu: telemetriyi çözer, seri porta basar, komut/filtre gönderir
├── iot_1/, iot_2/       # IoT istasyonları: sıcaklık verisini görev yüküne gönderir
2026-gorev-yuku/
├── gorevyuku_pcb_kod/   # Yeni PCB'ye uyarlanmış görev yükü (LoRa Serial8, GPS Serial7, buzzer pin 25)
├── yer_istasyonu/       # Güncellenmiş yer istasyonu (zaman aşımı ile komut gönderimi)
└── donanim/             # Şematik ve PCB çizimi (PDF)
```

## Derleme

Arduino IDE + Teensyduino ile derlenir (kart: **Teensy 4.1**). Gerekli kütüphaneler:

- Adafruit BME280, Adafruit BNO055, Adafruit Unified Sensor
- EByte LoRa E22 (`LoRa_E22.h`, Renzo Mischianti)
- TinyGPS++, PWMServo, TimeLib, SD

Her klasör ayrı bir Arduino sketch'idir; `.ino` dosyasını açıp ilgili karta yüklemek yeterlidir.
