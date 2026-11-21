#include "WString.h"
#ifndef UYDUSTAT_H
#define UYDUSTAT_H
#include <PWMServo.h>
extern PWMServo myservo3;

extern int32_t uydustatu;
extern float yukseklik1;
extern float tepe_noktasi;
extern int statu_sayac;
int Uydustatu_al() {
    switch (uydustatu) {
        case 0: // Uçuş öncesi
            if (yukseklik1 > 20) {
                statu_sayac++;
                if(statu_sayac==2){
                uydustatu = 1; // Yükselme
                }
            }
            break;

        case 1: // Yükselme
            if (tepe_noktasi-yukseklik1>20) {
                statu_sayac++;
                if(statu_sayac==4){
                uydustatu = 2; // Alçalma
                }
            }
            break;

        case 2: // Alçalma
            if ((yukseklik1<420) ) {
                statu_sayac++;
                if(statu_sayac==6){
                uydustatu = 3; // Ayrılma
                }

            }
            break;

        case 3: // Ayrılma
            if (yukseklik1 < 380) {
                statu_sayac++;
                if(statu_sayac==8){
                uydustatu = 4; // İniş
                }
            }
            break;

        case 4: // İniş
            if (yukseklik1 < 20) {
                statu_sayac++;
                if(statu_sayac==10){
                uydustatu = 5; // Kurtarma
                }
            }
            break;

        case 5:
            // Final state
            break;
    }

    return uydustatu;
}

#endif
