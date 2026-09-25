
#include <ESP32Servo.h>

// =====================================================
// ESP32 - CONTROLLER SERVO 6 DOF
// =====================================================
//
// J1 -> GPIO 23
// J2 -> GPIO 25
// J3 -> GPIO 26
// J4 -> GPIO 18
// J5 -> GPIO 13
// J6 -> GPIO 14
//
// =====================================================
// COMANDI
// =====================================================
//
// j1 0  -> 0°
// j1 1  -> 90°
// j1 2  -> 135°
// j1 3  -> 180°
// j1 4  -> 270°
// j1 5  -> 0° -> 180°
// j1 6  -> 0° -> 270°
//
// ANGOLI PERSONALIZZATI:
//
// j1 20g   -> 20°
// j1 45g   -> 45°
// j1 157g  -> 157°
// j1 230g  -> 230°
//
// Stessa cosa per J2-J6.
//
// =====================================================


// =====================================================
// PIN SERVO
// =====================================================

const int SERVO_PIN_J1 = 23;
const int SERVO_PIN_J2 = 25;
const int SERVO_PIN_J3 = 26;
const int SERVO_PIN_J4 = 18;
const int SERVO_PIN_J5 = 13;
const int SERVO_PIN_J6 = 14;


// =====================================================
// SERVO
// =====================================================

Servo servoJ1;
Servo servoJ2;
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
//
// J2 sostiene una parte del peso del braccio.
// Lo facciamo muovere più dolcemente.
//
// VELOCITA_J2:
// velocità massima di J2.
//
// POTENZA_J2:
// fattore di movimento.
//
// NON aumenta la coppia fisica del servo.
// Serve a rendere il movimento più controllato.
//
// =====================================================

int VELOCITA_J2 = 90;

float POTENZA_J2 = 0.60;


// =====================================================
// PWM
// =====================================================

const int PWM_MIN = 500;
const int PWM_MAX = 2500;


// =====================================================
// POSIZIONI ATTUALI
// =====================================================

float posizioneJ1 = 0;
float posizioneJ2 = 0;
float posizioneJ3 = 0;
float posizioneJ4 = 0;
float posizioneJ5 = 0;
float posizioneJ6 = 0;


// =====================================================
// CONVERSIONE ANGOLO -> PWM
// =====================================================

int angleToUs(float angle) {

  angle = constrain(
    angle,
    0,
    270
  );

  return PWM_MIN +
         (angle / 270.0) *
         (PWM_MAX - PWM_MIN);
}


// =====================================================
// MOVIMENTO NORMALE
// =====================================================

void muoviServoFluido(
  Servo &servo,
  float &posizione,
  float destinazione
) {

  destinazione =
    constrain(
      destinazione,
      0,
      270
    );


  while (true) {

    // -----------------------------------------------
    // DIFFERENZA
    // -----------------------------------------------

    float differenza =
      destinazione - posizione;


    // -----------------------------------------------
    // ARRIVATO
    // -----------------------------------------------

    if (abs(differenza) < 0.5) {

      posizione =
        destinazione;

      servo.writeMicroseconds(
        angleToUs(posizione)
      );

      break;
    }


    // -----------------------------------------------
    // VELOCITÀ
    // -----------------------------------------------

    float velocita =
      map(
        VELOCITA,
        0,
        100,
        1,
        30
      );


    // -----------------------------------------------
    // DISTANZA
    // -----------------------------------------------

    float distanza =
      abs(differenza);


    // -----------------------------------------------
    // RALLENTAMENTO
    // -----------------------------------------------

    float fattore = 1.0;


    if (distanza < 30) {

      fattore =
        distanza / 30.0;
    }


    // -----------------------------------------------
    // PASSO
    // -----------------------------------------------

    float passo =
      velocita *
      fattore *
      0.03;


    if (passo < 0.05) {

      passo = 0.05;
    }


    // -----------------------------------------------
    // MOVIMENTO
    // -----------------------------------------------

    if (differenza > 0) {

      posizione += passo;

      if (posizione > destinazione) {

        posizione =
          destinazione;
      }
    }

    else {

      posizione -= passo;

      if (posizione < destinazione) {

        posizione =
          destinazione;
      }
    }


    // -----------------------------------------------
    // PWM
    // -----------------------------------------------

    servo.writeMicroseconds(
      angleToUs(posizione)
    );


    delay(20);
  }


  posizione =
    destinazione;


  servo.writeMicroseconds(
    angleToUs(posizione)
  );
}


// =====================================================
// MOVIMENTO SPECIALE J2
// =====================================================
//
// J2 utilizza una curva più morbida:
//
// 1. Partenza lenta
// 2. Accelerazione graduale
// 3. Movimento controllato
// 4. Decelerazione graduale
//
// =====================================================

void muoviJ2(
  float destinazione
) {

  destinazione =
    constrain(
      destinazione,
      0,
      270
    );


  Serial.println(
    "Modalita J2 TORQUE ASSIST attiva."
  );


  while (true) {

    // -----------------------------------------------
    // DIFFERENZA
    // -----------------------------------------------

    float differenza =
      destinazione - posizioneJ2;


    // -----------------------------------------------
    // ARRIVO
    // -----------------------------------------------

    if (abs(differenza) < 0.5) {

      posizioneJ2 =
        destinazione;

      servoJ2.writeMicroseconds(
        angleToUs(posizioneJ2)
      );

      break;
    }


    // -----------------------------------------------
    // DISTANZA
    // -----------------------------------------------

    float distanza =
      abs(differenza);


    // -----------------------------------------------
    // VELOCITÀ BASE J2
    // -----------------------------------------------

    float velocita =
      map(
        VELOCITA_J2,
        0,
        100,
        1,
        30
      );


    // -----------------------------------------------
    // CURVA DI ACCELERAZIONE
    // -----------------------------------------------

    float accelerazione =
      1.0;


    // Primi 20 gradi:
    // partenza molto dolce

    if (distanza > 20) {

      accelerazione = 0.65;
    }


    // -----------------------------------------------
    // DECELERAZIONE
    // -----------------------------------------------

    if (distanza < 40) {

      accelerazione =
        distanza / 40.0;
    }


    // -----------------------------------------------
    // PASSO J2
    // -----------------------------------------------

    float passo =
      velocita *
      accelerazione *
      POTENZA_J2 *
      0.03;


    // -----------------------------------------------
    // PASSO MINIMO
    // -----------------------------------------------

    if (passo < 0.025) {

      passo = 0.025;
    }


    // -----------------------------------------------
    // MOVIMENTO
    // -----------------------------------------------

    if (differenza > 0) {

      posizioneJ2 += passo;


      if (
        posizioneJ2 >
        destinazione
      ) {

        posizioneJ2 =
          destinazione;
      }
    }

    else {

      posizioneJ2 -= passo;


      if (
        posizioneJ2 <
        destinazione
      ) {

        posizioneJ2 =
          destinazione;
      }
    }


    // -----------------------------------------------
    // INVIA PWM
    // -----------------------------------------------

    servoJ2.writeMicroseconds(
      angleToUs(posizioneJ2)
    );


    // -----------------------------------------------
    // TEMPO DI STABILIZZAZIONE
    // -----------------------------------------------

    delay(25);
  }


  // -----------------------------------------------
  // POSIZIONE FINALE
  // -----------------------------------------------

  posizioneJ2 =
    destinazione;


  servoJ2.writeMicroseconds(
    angleToUs(posizioneJ2)
  );


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

  destinazione =
    constrain(
      destinazione,
      0,
      270
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
    destinazione
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
      // 4 -> 270°
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
      // 6 -> 0° -> 270°
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
    // CONTROLLO RANGE
    // -----------------------------------------------

    if (
      angolo < 0 ||
      angolo > 270
    ) {

      Serial.println(
        "ERRORE: angolo tra 0 e 270."
      );

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
    PWM_MIN,
    PWM_MAX
  );

  servoJ2.attach(
    SERVO_PIN_J2,
    PWM_MIN,
    PWM_MAX
  );

  servoJ3.attach(
    SERVO_PIN_J3,
    PWM_MIN,
    PWM_MAX
  );

  servoJ4.attach(
    SERVO_PIN_J4,
    PWM_MIN,
    PWM_MAX
  );

  servoJ5.attach(
    SERVO_PIN_J5,
    PWM_MIN,
    PWM_MAX
  );

  servoJ6.attach(
    SERVO_PIN_J6,
    PWM_MIN,
    PWM_MAX
  );


  // =================================================
  // POSIZIONI INIZIALI
  // =================================================

  posizioneJ1 = 0;
  posizioneJ2 = 0;
  posizioneJ3 = 0;
  posizioneJ4 = 0;
  posizioneJ5 = 0;
  posizioneJ6 = 0;


  // =================================================
  // PORTA TUTTI A ZERO
  // =================================================

  servoJ1.writeMicroseconds(
    angleToUs(0)
  );

  servoJ2.writeMicroseconds(
    angleToUs(0)
  );

  servoJ3.writeMicroseconds(
    angleToUs(0)
  );

  servoJ4.writeMicroseconds(
    angleToUs(0)
  );

  servoJ5.writeMicroseconds(
    angleToUs(0)
  );

  servoJ6.writeMicroseconds(
    angleToUs(0)
  );


  // =================================================
  // INFORMAZIONI
  // =================================================

  Serial.println();

  Serial.println(
    "=========================================="
  );

  Serial.println(
    "        ESP32 6-DOF SERVO TEST"
  );

  Serial.println(
    "=========================================="
  );

  Serial.println();


  Serial.println(
    "PIN:"
  );

  Serial.println(
    "J1 -> GPIO 23"
  );

  Serial.println(
    "J2 -> GPIO 25"
  );

  Serial.println(
    "J3 -> GPIO 26"
  );

  Serial.println(
    "J4 -> GPIO 18"
  );

  Serial.println(
    "J5 -> GPIO 13"
  );

  Serial.println(
    "J6 -> GPIO 14"
  );


  Serial.println();


  // =================================================
  // COMANDI
  // =================================================

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
    "j1 4 -> 270°"
  );

  Serial.println(
    "j1 5 -> 0° -> 180°"
  );

  Serial.println(
    "j1 6 -> 0° -> 270°"
  );


  Serial.println();


  // =================================================
  // ANGOLO LIBERO
  // =================================================

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

  Serial.println(
    "j1 157g -> 157°"
  );

  Serial.println(
    "j1 230g -> 230°"
  );


  Serial.println();


  // =================================================
  // PARAMETRI J2
  // =================================================

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
