        
#include <ESP32Servo.h>

// =====================================================
// ESP32 - CONTROLLER SERVO 6 DOF - FINALE
// =====================================================
//
// J1 -> GPIO 23   (dispari, 270°, 500-2450µs)
// J2 -> GPIO 19   (pari,  180°, 500-1800µs) + GPIO 14 INVERSO
// J3 -> GPIO 25   (dispari, 270°, 500-2450µs)
// J4 -> GPIO 26   (pari,  180°, 500-1800µs)
// J5 -> GPIO 18   (dispari, 270°, 500-2450µs)
// J6 -> GPIO 13   (pari,  180°, 500-1800µs)
//
// =====================================================
// COMANDI
// =====================================================
//
// j1 0  -> 0°
// j1 1  -> 90°
// j1 2  -> 135°
// j1 3  -> 180°
// j1 4  -> 270° (solo giunti dispari)
// j1 5  -> 0° -> 180°
// j1 6  -> 0° -> 270° (solo giunti dispari)
//
// ANGOLI PERSONALIZZATI:
//
// j1 20g   -> 20°
// j1 45g   -> 45°
// j1 157g  -> 157°
// j1 230g  -> 230° (solo giunti dispari)
//
// Stessa cosa per J2-J6.
//
// =====================================================


// =====================================================
// PARAMETRI VELOCITÀ - MODIFICA QUI
// =====================================================
//
// VELOCITA: velocità generale (0-100)
// VELOCITA_J2: velocità J2 (0-100)
// POTENZA_J2: fattore potenza J2 (0.0-1.0)
// PASSO_MULTIPLIER: moltiplicatore passo (0.02=standard, 0.03=veloce)
// LOOP_DELAY: delay tra i passi in ms (20=standard, 15=veloce)
// SMOOTH_START: velocità iniziale (0.0=fermo, 0.3=reattivo, 1.0=parte a tutta)
//
// =====================================================

int VELOCITA_MASTER = 50;

int VELOCITA = 90;

int VELOCITA_J2 = 90;

int VELOCITA_RAMPA = 50;

float POTENZA_J2 = 0.60;

float PASSO_MULTIPLIER_BASE = 0.01;

int LOOP_DELAY_BASE = 8;

float SMOOTH_START = 0.3;

float J2_SLOW_ZONE = 40.0;
float J2_SLOW_FACTOR = 0.15;


// =====================================================
// VARIABILI CALCOLATE DA VELOCITA_MASTER
// =====================================================

float PASSO_MULTIPLIER;
int LOOP_DELAY;
float RAMPA_EXP;


// =====================================================
// PIN SERVO
// =====================================================

const int SERVO_PIN_J1 = 23;
const int SERVO_PIN_J2 = 19;
const int SERVO_PIN_J2_INV = 14;
const int SERVO_PIN_J3 = 25;
const int SERVO_PIN_J4 = 26;
const int SERVO_PIN_J5 = 18;
const int SERVO_PIN_J6 = 13;


// =====================================================
// SERVO
// =====================================================

Servo servoJ1;
Servo servoJ2;
Servo servoJ2Inv;
Servo servoJ3;
Servo servoJ4;
Servo servoJ5;
Servo servoJ6;


// =====================================================
// PWM CALIBRAZIONE DS3235
// =====================================================
//
// Dispari (J1, J3, J5): 270° -> 500-2450µs
// Pari (J2, J4, J6): 180° -> 500-1800µs
//
// 500µs = 0°
// 1800µs = 180°
// 2450µs = 270°
//
// =====================================================

const int PWM_MIN_ODD = 500;
const int PWM_MAX_ODD = 2450;

const int PWM_MIN_EVEN = 500;
const int PWM_MAX_EVEN = 1800;


// =====================================================
// POSIZIONI ATTUALI
// =====================================================

float posizioneJ1 = 135;
float posizioneJ2 = 90;
float posizioneJ3 = 135;
float posizioneJ4 = 90;
float posizioneJ5 = 135;
float posizioneJ6 = 90;


// =====================================================
// FUNZIONI UTILITÀ
// =====================================================

float getMaxAngle(int numeroServo) {
  if (numeroServo % 2 == 1) {
    return 270.0;
  } else {
    return 180.0;
  }
}


// =====================================================
// SMOOTHSTEP - CURVA C2 CONTINUA
// =====================================================
//
// smooth(t) = 3t² - 2t³
//
// t=0 -> 0 (fermo)
// t=0.5 -> 0.5 (velocità massima)
// t=1 -> 0 (fermo)
//
// Derivata continua = nessun scatto
//
// =====================================================

float smoothstep(float t) {
  t = constrain(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}


// =====================================================
// CONVERSIONE ANGOLO -> PWM
// =====================================================

int angleToUs(int numeroServo, float angle) {

  if (numeroServo % 2 == 1) {

    // Dispari: 270°
    angle = constrain(angle, 0, 270);

    return PWM_MIN_ODD +
           (angle / 270.0) *
           (PWM_MAX_ODD - PWM_MIN_ODD);

  } else {

    // Pari: 180°
    angle = constrain(angle, 0, 180);

    return PWM_MIN_EVEN +
           (angle / 180.0) *
           (PWM_MAX_EVEN - PWM_MIN_EVEN);
  }
}


// =====================================================
// CONVERSIONE ANGOLO -> PWM INVERSO (J2)
// =====================================================

int angleToUsInv(float angle) {

  angle = constrain(angle, 0, 180);

  return PWM_MIN_EVEN +
         ((180.0 - angle) / 180.0) *
         (PWM_MAX_EVEN - PWM_MIN_EVEN);
}


// =====================================================
// MOVIMENTO FLUIDO CON S-CURVE
// =====================================================

void muoviServoFluido(
  Servo &servo,
  float &posizione,
  float destinazione,
  int numeroServo
) {

  float maxAngle = getMaxAngle(numeroServo);

  destinazione =
    constrain(
      destinazione,
      0,
      maxAngle
    );


  // -----------------------------------------------
  // CALCOLA DISTANZA TOTALE
  // -----------------------------------------------

  float start = posizione;
  float totalDist = destinazione - start;
  float distanzaTotale = abs(totalDist);


  // -----------------------------------------------
  // SE GIA ARRIVATO, ESCI
  // -----------------------------------------------

  if (distanzaTotale < 0.5) {

    posizione = destinazione;

    servo.writeMicroseconds(
      angleToUs(numeroServo, posizione)
    );

    return;
  }


  // -----------------------------------------------
  // DIREZIONE (+1 o -1)
  // -----------------------------------------------

  float direzione = (totalDist > 0) ? 1.0 : -1.0;


  // -----------------------------------------------
  // LOOP MOVIMENTO
  // -----------------------------------------------

  while (true) {


    // -------------------------------------------
    // CALCOLA PROGRESSO (0.0 -> 1.0)
    // -------------------------------------------

    float distanzaPercorsa = abs(posizione - start);
    float progresso = distanzaPercorsa / distanzaTotale;
    progresso = constrain(progresso, 0.0, 1.0);


    // -------------------------------------------
    // S-CURVE: VELCOCITÀ MORBIDA
    // (parte al 30% per evitare partenza lenta)
    // -------------------------------------------

    float vel = SMOOTH_START + (1.0 - SMOOTH_START) * smoothstep(pow(progresso, RAMPA_EXP));


    // -------------------------------------------
    // VELOCITÀ MASSIMA
    // -------------------------------------------

    float velocitaMax = map(
      VELOCITA,
      0,
      100,
      5,
      100
    );


    // -------------------------------------------
    // VELOCITÀ ATTUALE
    // -------------------------------------------

    float velocita = velocitaMax * vel;


    // -------------------------------------------
    // VELOCITÀ MINIMA
    // -------------------------------------------

    if (velocita < 0.3) {

      velocita = 0.3;
    }


    // -------------------------------------------
    // CALCOLA PASSO
    // -------------------------------------------

    float passo = velocita * PASSO_MULTIPLIER;


    // -------------------------------------------
    // NON SUPERARE LA DESTINAZIONE
    // -------------------------------------------

    float distanzaRimanente = abs(destinazione - posizione);

    if (passo > distanzaRimanente) {

      passo = distanzaRimanente;
    }


    // -------------------------------------------
    // APPLICA PASSO
    // -------------------------------------------

    posizione += direzione * passo;


    // -------------------------------------------
    // CONTROLLA ARRIVO
    // -------------------------------------------

    if (abs(posizione - destinazione) < 0.5) {

      posizione = destinazione;

      servo.writeMicroseconds(
        angleToUs(numeroServo, posizione)
      );

      break;
    }


    // -------------------------------------------
    // INVIA PWM
    // -------------------------------------------

    servo.writeMicroseconds(
      angleToUs(numeroServo, posizione)
    );


    // -------------------------------------------
    //Delay
    // -------------------------------------------

    delay(LOOP_DELAY);
  }
}


// =====================================================
// MOVIMENTO J2 FLUIDO CON SYNC
// =====================================================

void muoviJ2(
  float destinazione
) {

  destinazione =
    constrain(
      destinazione,
      0,
      180
    );


  Serial.println(
    "Modalita J2 FLUIDO + SYNC attiva."
  );


  // -----------------------------------------------
  // CALCOLA DISTANZA TOTALE
  // -----------------------------------------------

  float start = posizioneJ2;
  float totalDist = destinazione - start;
  float distanzaTotale = abs(totalDist);


  // -----------------------------------------------
  // SE GIA ARRIVATO, ESCI
  // -----------------------------------------------

  if (distanzaTotale < 0.5) {

    posizioneJ2 = destinazione;

    int pwmN = angleToUs(2, posizioneJ2);
    int pwmI = angleToUsInv(posizioneJ2);

    servoJ2.writeMicroseconds(pwmN);
    servoJ2Inv.writeMicroseconds(pwmI);

    return;
  }


  // -----------------------------------------------
  // DIREZIONE
  // -----------------------------------------------

  float direzione = (totalDist > 0) ? 1.0 : -1.0;


  // -----------------------------------------------
  // LOOP MOVIMENTO J2
  // -----------------------------------------------

  while (true) {


    // -------------------------------------------
    // PROGRESSO (0.0 -> 1.0)
    // -------------------------------------------

    float distanzaPercorsa = abs(posizioneJ2 - start);
    float progresso = distanzaPercorsa / distanzaTotale;
    progresso = constrain(progresso, 0.0, 1.0);


    // -------------------------------------------
    // S-CURVE (parte al 30% per partenza reattiva)
    // -------------------------------------------

    float vel = SMOOTH_START + (1.0 - SMOOTH_START) * smoothstep(pow(progresso, RAMPA_EXP));


    // -------------------------------------------
    // VELOCITÀ MASSIMA J2
    // -------------------------------------------

    float velocitaMax = map(
      VELOCITA_J2,
      0,
      100,
      3,
      70
    );


    // -------------------------------------------
    // APPLICA POTENZA J2
    // -------------------------------------------

    float velocita = velocitaMax * vel * POTENZA_J2;


    // -------------------------------------------
    // ZONA LENTA: SOTTO 40° E SOPRA 140°
    // -------------------------------------------

    if (posizioneJ2 < J2_SLOW_ZONE) {

      float factor = posizioneJ2 / J2_SLOW_ZONE;
      velocita *= J2_SLOW_FACTOR + factor * (1.0 - J2_SLOW_FACTOR);

    } else if (posizioneJ2 > (180.0 - J2_SLOW_ZONE)) {

      float factor = (180.0 - posizioneJ2) / J2_SLOW_ZONE;
      velocita *= J2_SLOW_FACTOR + factor * (1.0 - J2_SLOW_FACTOR);
    }


    // -------------------------------------------
    // VELOCITÀ MINIMA
    // -------------------------------------------

    if (velocita < 0.3) {

      velocita = 0.3;
    }


    // -------------------------------------------
    // CALCOLA PASSO
    // -------------------------------------------

    float passo = velocita * PASSO_MULTIPLIER;


    // -------------------------------------------
    // NON SUPERARE DESTINAZIONE
    // -------------------------------------------

    float distanzaRimanente = abs(destinazione - posizioneJ2);

    if (passo > distanzaRimanente) {

      passo = distanzaRimanente;
    }


    // -------------------------------------------
    // APPLICA PASSO
    // -------------------------------------------

    posizioneJ2 += direzione * passo;


    // -------------------------------------------
    // ARRIVO
    // -------------------------------------------

    if (abs(posizioneJ2 - destinazione) < 0.5) {

      posizioneJ2 = destinazione;

      int pwmN = angleToUs(2, posizioneJ2);
      int pwmI = angleToUsInv(posizioneJ2);

      servoJ2.writeMicroseconds(pwmN);
      servoJ2Inv.writeMicroseconds(pwmI);

      break;
    }


    // -------------------------------------------
    // SYNC: INVIA PWM A ENTRAMBI I MOTORI
    // NELLO STESSO ISTANTE
    // -------------------------------------------

    int pwmNormale = angleToUs(2, posizioneJ2);
    int pwmInverso = angleToUsInv(posizioneJ2);

    servoJ2.writeMicroseconds(pwmNormale);
    servoJ2Inv.writeMicroseconds(pwmInverso);


    // -------------------------------------------
    //Delay
    // -------------------------------------------

    delay(LOOP_DELAY);
  }


  // -----------------------------------------------
  // POSIZIONE FINALE
  // -----------------------------------------------

  posizioneJ2 = destinazione;

  int pwmN = angleToUs(2, posizioneJ2);
  int pwmI = angleToUsInv(posizioneJ2);

  servoJ2.writeMicroseconds(pwmN);
  servoJ2Inv.writeMicroseconds(pwmI);


  Serial.print(
    "J2 arrivato a "
  );

  Serial.print(
    destinazione
  );

  Serial.println(
    " gradi."
  );
}


// =====================================================
// VAI A POSIZIONE
// =====================================================

void vaiA(
  Servo &servo,
  float &posizione,
  float destinazione,
  int numeroServo
) {

  float maxAngle = getMaxAngle(numeroServo);

  destinazione =
    constrain(
      destinazione,
      0,
      maxAngle
    );


  // ---------------------------------------------------
  // J2 USA IL SISTEMA SPECIALE
  // ---------------------------------------------------

  if (numeroServo == 2) {

    muoviJ2(
      destinazione
    );

    return;
  }


  // ---------------------------------------------------
  // ALTRI SERVO
  // ---------------------------------------------------

  Serial.print(
    "J"
  );

  Serial.print(
    numeroServo
  );

  Serial.print(
    " -> "
  );

  Serial.print(
    destinazione
  );

  Serial.println(
    " gradi"
  );


  muoviServoFluido(
    servo,
    posizione,
    destinazione,
    numeroServo
  );


  Serial.print(
    "J"
  );

  Serial.print(
    numeroServo
  );

  Serial.println(
    " arrivato."
  );
}


// =====================================================
// MOVIMENTO DA ZERO
// =====================================================

void movimentoDaZero(
  Servo &servo,
  float &posizione,
  float destinazione,
  int numeroServo
) {

  Serial.print(
    "J"
  );

  Serial.print(
    numeroServo
  );

  Serial.print(
    " : 0 -> "
  );

  Serial.print(
    destinazione
  );

  Serial.println(
    " gradi"
  );


  // -----------------------------------------------
  // TORNA A ZERO
  // -----------------------------------------------

  vaiA(
    servo,
    posizione,
    0,
    numeroServo
  );


  delay(500);


  // -----------------------------------------------
  // VA ALLA DESTINAZIONE
  // -----------------------------------------------

  vaiA(
    servo,
    posizione,
    destinazione,
    numeroServo
  );
}


// =====================================================
// ESEGUI COMANDO
// =====================================================

void eseguiComando(
  int numeroServo,
  String comando
) {

  Servo *servo = nullptr;

  float *posizione = nullptr;


  // =================================================
  // SELEZIONA SERVO
  // =================================================

  switch (numeroServo) {

    case 1:

      servo = &servoJ1;
      posizione = &posizioneJ1;

      break;


    case 2:

      servo = &servoJ2;
      posizione = &posizioneJ2;

      break;


    case 3:

      servo = &servoJ3;
      posizione = &posizioneJ3;

      break;


    case 4:

      servo = &servoJ4;
      posizione = &posizioneJ4;

      break;


    case 5:

      servo = &servoJ5;
      posizione = &posizioneJ5;

      break;


    case 6:

      servo = &servoJ6;
      posizione = &posizioneJ6;

      break;


    default:

      Serial.println(
        "ERRORE: servo non valido."
      );

      return;
  }


  // =================================================
  // COMANDI 0-6 (TEST MIN/MAX)
  // =================================================

  if (
    comando == "0" ||
    comando == "1" ||
    comando == "2" ||
    comando == "3" ||
    comando == "4" ||
    comando == "5" ||
    comando == "6"
  ) {

    int numeroComando = comando.toInt();
    bool isPari = (numeroServo % 2 == 0);
    float centro = isPari ? 90.0 : 135.0;
    float maxAngolo = isPari ? 180.0 : 270.0;

    switch (numeroComando) {

      // MIN -> 0°
      case 0:

        vaiA(*servo, *posizione, 0, numeroServo);
        break;


      // CENTRO
      case 1:

        vaiA(*servo, *posizione, centro, numeroServo);
        break;


      // MAX
      case 2:

        vaiA(*servo, *posizione, maxAngolo, numeroServo);
        break;


      // CENTRO -> MIN -> CENTRO
      case 3:

        vaiA(*servo, *posizione, centro, numeroServo);
        vaiA(*servo, *posizione, 0, numeroServo);
        vaiA(*servo, *posizione, centro, numeroServo);
        break;


      // CENTRO -> MAX -> CENTRO
      case 4:

        vaiA(*servo, *posizione, centro, numeroServo);
        vaiA(*servo, *posizione, maxAngolo, numeroServo);
        vaiA(*servo, *posizione, centro, numeroServo);
        break;


      // MIN -> MAX -> MIN
      case 5:

        vaiA(*servo, *posizione, 0, numeroServo);
        vaiA(*servo, *posizione, maxAngolo, numeroServo);
        vaiA(*servo, *posizione, 0, numeroServo);
        break;


      // CENTRO -> MIN -> MAX -> CENTRO
      case 6:

        vaiA(*servo, *posizione, centro, numeroServo);
        vaiA(*servo, *posizione, 0, numeroServo);
        vaiA(*servo, *posizione, maxAngolo, numeroServo);
        vaiA(*servo, *posizione, centro, numeroServo);
        break;
    }

    printHelp();
    return;
  }


  // =================================================
  // ANGOLO PERSONALIZZATO
  // =================================================

  if (
    comando.endsWith("g")
  ) {

    String numero =
      comando.substring(
        0,
        comando.length() - 1
      );


    float angolo =
      numero.toFloat();


    // -----------------------------------------------
    // CONTROLLO RANGE PER TIPO GIUNTO
    // -----------------------------------------------

    float maxAngle = getMaxAngle(numeroServo);

    if (
      angolo < 0 ||
      angolo > maxAngle
    ) {

      Serial.println(
        "ERRORE: angolo fuori range per questo giunto."
      );

      Serial.print(
        "Range: 0 - "
      );

      Serial.println(maxAngle);

      return;
    }


    // -----------------------------------------------
    // MOVIMENTO
    // -----------------------------------------------

    vaiA(
      *servo,
      *posizione,
      angolo,
      numeroServo
    );

    printHelp();
    return;
  }


  // =================================================
  // ERRORE
  // =================================================

  Serial.println(
    "ERRORE: comando non riconosciuto."
  );

  Serial.println(
    "Usa 0-6 oppure un angolo come 20g."
  );
}


// =====================================================
// STAMPA COMANDI
// =====================================================

void printHelp() {

  Serial.println();
  Serial.println("==========================================");
  Serial.println("           COMANDI DISPONIBILI");
  Serial.println("==========================================");
  Serial.println();
  Serial.println("jX 0  -> Vai a MIN (0°)");
  Serial.println("jX 1  -> Vai a CENTRO");
  Serial.println("jX 2  -> Vai a MAX");
  Serial.println("jX 3  -> CENTRO -> MIN -> CENTRO");
  Serial.println("jX 4  -> CENTRO -> MAX -> CENTRO");
  Serial.println("jX 5  -> MIN -> MAX -> MIN");
  Serial.println("jX 6  -> CENTRO -> MIN -> MAX -> CENTRO");
  Serial.println();
  Serial.println("jX Yg -> Vai a Y° (es: j1 45g)");
  Serial.println();
  Serial.println("Pari (J2,J4,J6): CENTRO=90°  MAX=180°");
  Serial.println("Dispari (J1,J3,J5): CENTRO=135°  MAX=270°");
  Serial.println();
  Serial.println("==========================================");
  Serial.println();
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(500);


  // =================================================
  // CALCOLA PARAMETRI DA VELOCITA_MASTER
  // =================================================

  float master = constrain(VELOCITA_MASTER, 0, 100);

  float multiplier = pow(2.0, (master - 50) / 25.0);

  PASSO_MULTIPLIER = PASSO_MULTIPLIER_BASE * multiplier;
  LOOP_DELAY = max(1, (int)(LOOP_DELAY_BASE / multiplier));

  float rampa = constrain(VELOCITA_RAMPA, 0, 100);
  RAMPA_EXP = 1.0 + 2.0 * (1.0 - rampa / 50.0);
  RAMPA_EXP = constrain(RAMPA_EXP, 0.3, 3.0);


  // =================================================
  // ATTACCA SERVO
  // =================================================

  servoJ1.attach(
    SERVO_PIN_J1,
    PWM_MIN_ODD,
    PWM_MAX_ODD
  );

  servoJ2.attach(
    SERVO_PIN_J2,
    PWM_MIN_EVEN,
    PWM_MAX_EVEN
  );

  servoJ2Inv.attach(
    SERVO_PIN_J2_INV,
    PWM_MIN_EVEN,
    PWM_MAX_EVEN
  );

  servoJ3.attach(
    SERVO_PIN_J3,
    PWM_MIN_ODD,
    PWM_MAX_ODD
  );

  servoJ4.attach(
    SERVO_PIN_J4,
    PWM_MIN_EVEN,
    PWM_MAX_EVEN
  );

  servoJ5.attach(
    SERVO_PIN_J5,
    PWM_MIN_ODD,
    PWM_MAX_ODD
  );

  servoJ6.attach(
    SERVO_PIN_J6,
    PWM_MIN_EVEN,
    PWM_MAX_EVEN
  );


  // =================================================
  // PORTA TUTTI ALLA POSIZIONE INIZIALE
  // =================================================

  servoJ1.writeMicroseconds(
    angleToUs(1, 135)
  );

  servoJ2.writeMicroseconds(
    angleToUs(2, 90)
  );

  servoJ2Inv.writeMicroseconds(
    angleToUsInv(90)
  );

  servoJ3.writeMicroseconds(
    angleToUs(3, 135)
  );

  servoJ4.writeMicroseconds(
    angleToUs(4, 90)
  );

  servoJ5.writeMicroseconds(
    angleToUs(5, 135)
  );

  servoJ6.writeMicroseconds(
    angleToUs(6, 90)
  );


  // =================================================
  // INFORMAZIONI
  // =================================================

  Serial.println();

  Serial.println(
    "=========================================="
  );

  Serial.println(
    "  ESP32 6-DOF SERVO - FINALE"
  );

  Serial.println(
    "=========================================="
  );

  Serial.println();


  Serial.println(
    "PIN:"
  );

  Serial.println(
    "J1 -> GPIO 23  (270°)"
  );

  Serial.println(
    "J2 -> GPIO 19  (180°) + GPIO 14 (INVERSO)"
  );

  Serial.println(
    "J3 -> GPIO 25  (270°)"
  );

  Serial.println(
    "J4 -> GPIO 26  (180°)"
  );

  Serial.println(
    "J5 -> GPIO 18  (270°)"
  );

  Serial.println(
    "J6 -> GPIO 13  (180°)"
  );


  Serial.println();


  Serial.println(
    "MOVIMENTO: S-Curve (nessun scatto)"
  );

  Serial.println(
    "J2: Doppio motore SYNC"
  );


  Serial.println();


  Serial.println(
    "=== PARAMETRI VELOCITÀ ==="
  );

  Serial.println(
    "(modifica nella sezione PARAMETRI VELOCITÀ)"
  );

  Serial.println();

  Serial.print("VELOCITA = ");
  Serial.println(VELOCITA);

  Serial.print("VELOCITA_J2 = ");
  Serial.println(VELOCITA_J2);

  Serial.print("POTENZA_J2 = ");
  Serial.println(POTENZA_J2);

  Serial.print("PASSO_MULTIPLIER = ");
  Serial.println(PASSO_MULTIPLIER);

  Serial.print("LOOP_DELAY = ");
  Serial.print(LOOP_DELAY);
  Serial.println("ms");


  Serial.println();
  printHelp();
  Serial.println("ESP32 PRONTO.");
  Serial.println();
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  if (Serial.available()) {


    // =================================================
    // LEGGE RIGA
    // =================================================

    String input =
      Serial.readStringUntil('\n');


    // =================================================
    // PULIZIA
    // =================================================

    input.trim();

    input.toLowerCase();


    if (
      input.length() == 0
    ) {

      return;
    }


    // =================================================
    // TROVA SERVO
    // =================================================

    int numeroServo = 0;


    if (
      input.startsWith("j1")
    ) {

      numeroServo = 1;
    }

    else if (
      input.startsWith("j2")
    ) {

      numeroServo = 2;
    }

    else if (
      input.startsWith("j3")
    ) {

      numeroServo = 3;
    }

    else if (
      input.startsWith("j4")
    ) {

      numeroServo = 4;
    }

    else if (
      input.startsWith("j5")
    ) {

      numeroServo = 5;
    }

    else if (
      input.startsWith("j6")
    ) {

      numeroServo = 6;
    }

    else {

      Serial.println(
        "ERRORE: servo non valido."
      );

      return;
    }


    // =================================================
    // TROVA SPAZIO
    // =================================================

    int spazio =
      input.indexOf(' ');


    if (
      spazio == -1
    ) {

      Serial.println(
        "FORMATO: j1 90g"
      );

      return;
    }


    // =================================================
    // ESTRAE COMANDO
    // =================================================

    String comando =
      input.substring(
        spazio + 1
      );


    comando.trim();


    // =================================================
    // ESEGUI
    // =================================================

    eseguiComando(
      numeroServo,
      comando
    );
  }
}
