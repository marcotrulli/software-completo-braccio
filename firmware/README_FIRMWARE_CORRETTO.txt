FIRMWARE CORRETTO PER ESP32 - USARE QUESTO

File: Robot6DOF_ESP32_FINALE_CORRETTO.ino
Sorgente: C:\Users\Marco Trulli\Desktop\test_servo\robot6dof_final_tinker\robot6dof_final_tinker.ino (569 righe)
Mappatura: J1 23 / J2 19+14inv / J3 25 / J4 26 / J5 18 / J6 13
PWM: odd 500-2450 (270�) even 500-1800 (180�)
UART: Serial (USB COM5) + Serial2 16/17 @115200
Fix: no scatto, J2 invertito, CAL, stato ogni 80ms

Aprire SOLO questo .ino in Arduino IDE -> ESP32 Dev Module -> COM5 -> Upload

STORICO FLASH
-------------
22.09.26 - Flashato firmware ESP32 da:
  C:\Users\Marco Trulli\Desktop\Robot6DOF_ESP32_FLASH\Robot6DOF_ESP32_FINALE_CORRETTO.ino
  (copia di: firmware\Robot6DOF_ESP32_FINALE_CORRETTO\Robot6DOF_ESP32_FINALE_CORRETTO.ino)
  Fix inclusi: filtro CAL in loop(), handleCal non-bloccante con state continuo.

ISTRUZIONE PER PROSSIMO AGENTE
------------------------------
Per ogni nuovo flash del firmware ESP32, aggiornare SEMPRE questo README:
aggiungere una riga sotto "STORICO FLASH" con data e percorso completo del file .ino flashato.


