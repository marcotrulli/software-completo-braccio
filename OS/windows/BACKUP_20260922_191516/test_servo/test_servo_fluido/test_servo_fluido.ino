
#include <ESP32Servo.h>

// =====================================================
// ESP32 - CONTROLLER SERVO 6 DOF - FLUIDO
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
// VELOCITÀ GENERALE
// =====================================================

// Da 0 a 100

int VELOCITA = 70;


// =====================================================
// PARAMETRI SPECIALI J2
// =====================================================

int VELOCITA_J2 = 90;

float POTENZA_J2 = 0.60;


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
//
// Profilo di velocità:
//
// 1. Partenza lenta (smoothstep cresce da 0)
// 2. Accelerazione graduale
// 3. Velocità massima a metà percorrenza
// 4. Decelerazione graduale
// 5. Arrivo lenta (smoothstep torna a 0)
//
// Nessuno scatto perché la curva è C2 continua.
//
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
    // -------------------------------------------

    float vel = smoothstep(progresso);


    // -------------------------------------------
    // VELOCITÀ MASSIMA
    // -------------------------------------------

    float velocitaMax = map(
      VELOCITA,
      0,
      100,
      5,
      50
    );


    // -------------------------------------------
    // VELOCITÀ ATTUALE
    // -------------------------------------------

    float velocita = velocitaMax * vel;


    // -------------------------------------------
    // VELOCITÀ MINIMA
    // (per garantire movimento reale)
    // -------------------------------------------

    if (velocita < 1.0) {

      velocita = 1.0;
    }


    // -------------------------------------------
    // CALCOLA PASSO
    // -------------------------------------------

    float passo = velocita * 0.02;


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

    delay(20);
  }
}


// =====================================================
// MOVIMENTO J2 FLUIDO CON SYNC
// =====================================================
//
// J2 è un giunto pari: range 0-180°
// Utilizza S-curve per movimento morbido
// Invia PWM a entrambi i motori nello stesso istante
//
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
    // S-CURVE
    // -------------------------------------------

    float vel = smoothstep(progresso);


    // -------------------------------------------
    // VELOCITÀ MASSIMA J2
    // -------------------------------------------

    float velocitaMax = map(
      VELOCITA_J2,
      0,
      100,
      3,
      35
    );


    // -------------------------------------------
    // APPLICA POTENZA J2
    // -------------------------------------------

    float velocita = velocitaMax * vel * POTENZA_J2;


    // -------------------------------------------
    // VELOCITÀ MINIMA
    // -------------------------------------------

    if (velocita < 0.8) {

      velocita = 0.8;
    }


    // -------------------------------------------
    // CALCOLA PASSO
    // -------------------------------------------

    float passo = velocita * 0.02;


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

    delay(20);
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
  // COMANDI 0-6
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

    int numeroComando =
      comando.toInt();


    // -------------------------------------------------
    // VALIDAZIONE RANGE PER TIPO GIUNTO
    // -------------------------------------------------

    bool isPari = (numeroServo % 2 == 0);


    // Comando 4 -> 270°: solo per dispari
    if (numeroComando == 4 && isPari) {

      Serial.println(
        "ERRORE: J"
      );

      Serial.print(numeroServo);

      Serial.println(
        " e un giunto pari (max 180°)."
      );

      return;
    }


    // Comando 6 -> 0° -> 270°: solo per dispari
    if (numeroComando == 6 && isPari) {

      Serial.println(
        "ERRORE: J"
      );

      Serial.print(numeroServo);

      Serial.println(
        " e un giunto pari (max 180°)."
      );

      return;
    }


    switch (numeroComando) {

      // ---------------------------------------------
      // 0 -> 0°
      // ---------------------------------------------

      case 0:

        vaiA(
          *servo,
          *posizione,
          0,
          numeroServo
        );

        break;


      // ---------------------------------------------
      // 1 -> 90°
      // ---------------------------------------------

      case 1:

        vaiA(
          *servo,
          *posizione,
          90,
          numeroServo
        );

        break;


      // ---------------------------------------------
      // 2 -> 135°
      // ---------------------------------------------

      case 2:

        vaiA(
          *servo,
          *posizione,
          135,
          numeroServo
        );

        break;


      // ---------------------------------------------
      // 3 -> 180°
      // ---------------------------------------------

      case 3:

        vaiA(
          *servo,
          *posizione,
          180,
          numeroServo
        );

        break;


      // ---------------------------------------------
      // 4 -> 270° (solo dispari)
      // ---------------------------------------------

      case 4:

        vaiA(
          *servo,
          *posizione,
          270,
          numeroServo
        );

        break;


      // ---------------------------------------------
      // 5 -> 0° -> 180°
      // ---------------------------------------------

      case 5:

        movimentoDaZero(
          *servo,
          *posizione,
          180,
          numeroServo
        );

        break;


      // ---------------------------------------------
      // 6 -> 0° -> 270° (solo dispari)
      // ---------------------------------------------

      case 6:

        movimentoDaZero(
          *servo,
          *posizione,
          270,
          numeroServo
        );

        break;
    }

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
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(500);


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
    "  ESP32 6-DOF SERVO - FLUIDO + SYNC"
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
    "COMANDI:"
  );

  Serial.println();

  Serial.println(
    "j1 0 -> 0°"
  );

  Serial.println(
    "j1 1 -> 90°"
  );

  Serial.println(
    "j1 2 -> 135°"
  );

  Serial.println(
    "j1 3 -> 180°"
  );

  Serial.println(
    "j1 4 -> 270° (solo dispari)"
  );

  Serial.println(
    "j1 5 -> 0° -> 180°"
  );

  Serial.println(
    "j1 6 -> 0° -> 270° (solo dispari)"
  );


  Serial.println();


  Serial.println(
    "ANGOLO PERSONALIZZATO:"
  );

  Serial.println();

  Serial.println(
    "j1 20g -> 20°"
  );

  Serial.println(
    "j1 45g -> 45°"
  );


  Serial.println();


  Serial.println(
    "PARAMETRI J2:"
  );

  Serial.print(
    "VELOCITA_J2 = "
  );

  Serial.println(
    VELOCITA_J2
  );

  Serial.print(
    "POTENZA_J2 = "
  );

  Serial.println(
    POTENZA_J2
  );


  Serial.println();


  Serial.print(
    "VELOCITA GENERALE = "
  );

  Serial.println(
    VELOCITA
  );


  Serial.println();

  Serial.println(
    "POSIZIONI INIZIALI: J1=135° J2=90° J3=135° J4=90° J5=135° J6=90°"
  );


  Serial.println();

  Serial.println(
    "ESP32 PRONTO."
  );

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
