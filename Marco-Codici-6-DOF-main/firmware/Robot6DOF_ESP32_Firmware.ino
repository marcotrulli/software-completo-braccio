/* =========================================================================
   ROBOT 6 DOF - FIRMWARE PROFESSIONALE COMPLETO PER ESP32
   =========================================================================
   Generato da Digital Twin 6 DOF Workspace Studio (Marco Codici 6 DOF)
   
   Caratteristiche incluse:
   1. Cinematica Inversa 6 DOF Ibrida (Analitica 3-Link + DLS 6 DOF)
   2. Postura a Minimo Stress J2 (Gru / Crane Posture & Anti-Sovraccarico)
   3. Vincolo Pinza Orizzontale & Radiale
   4. Pianificatore di Traiettoria Morbida (Cubic S-Curve & Anti-Stall Waypoints)
   5. Parser Seriale Interattivo a 115200 baud (Comandi: IK, MOVE, REST, STATUS...)
   6. Supporto sia per Pin PWM Diretti ESP32 che per Driver I2C PCA9685
   ========================================================================= */

#include <Arduino.h>
#include <math.h>

// =========================================================================
// 1. CONFIGURAZIONE HARDWARE & PIN GPIO ESP32
// =========================================================================
// Seleziona la modalità di controllo servomotori:
// - 1: Controllo Diretto tramite GPIO ESP32 (Libreria ESP32Servo)
// - 0: Controllo tramite modulo I2C PCA9685 16 Canali (Wire / Adafruit_PWMServoDriver)
#define USE_DIRECT_ESP32_GPIO 1

#if USE_DIRECT_ESP32_GPIO
  #include <ESP32Servo.h>
  
  // Assegnazione Pin GPIO per i 6 Servomotori (Modifica secondo i tuoi collegamenti)
  const int SERVO_PIN_J1 = 13; // Base Rotazione (Yaw 270°)
  const int SERVO_PIN_J2 = 12; // Spalla Pitch (180°)
  const int SERVO_PIN_J3 = 14; // Torsione Braccio Roll (270°)
  const int SERVO_PIN_J4 = 27; // Gomito Pitch (180°)
  const int SERVO_PIN_J5 = 26; // Torsione Polso Roll (270°)
  const int SERVO_PIN_J6 = 25; // Inclinazione Pinza Pitch (180°)
  
  Servo servos[6];
  const int servoPins[6] = {SERVO_PIN_J1, SERVO_PIN_J2, SERVO_PIN_J3, SERVO_PIN_J4, SERVO_PIN_J5, SERVO_PIN_J6};
#else
  #include <Wire.h>
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
  const int servoChannels[6] = {0, 1, 2, 3, 4, 5};
#endif

// =========================================================================
// 2. CALIBRAZIONE IMPULSI PWM (Microsecondi) & LIMITI MECCANICI GIUNTI
// =========================================================================
// Microsecondi minimi e massimi per i servi (Generalmente 500us = 0°, 2500us = Max)
const int PWM_MIN_US[6] = { 500,  500,  500,  500,  500,  500};
const int PWM_MAX_US[6] = {2500, 2500, 2500, 2500, 2500, 2500};

// Offset di calibrazione meccanica in gradi (per centrare perfettamente a 0°)
float jointAngleOffsets[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Limiti software angolari (in gradi cinematici):
// J1 (dispari 270°): [-135, +135] -> span 270°
// J2 (pari 180°):    [-90, +90]   -> span 180°
// J3 (dispari 270°): [-135, +135] -> span 270°
// J4 (pari 180°):    [-90, +90]   -> span 180° (flessione gomito fino a -90°)
// J5 (dispari 270°): [-135, +135] -> span 270°
// J6 (pari 180°):    [-90, +90]   -> span 180°
const float Q_MIN[6] = {-135.0f, -90.0f, -135.0f, -90.0f, -135.0f, -90.0f};
const float Q_MAX[6] = { 135.0f,  90.0f,  135.0f,  90.0f,  135.0f,  90.0f};

// Velocità massima giunti (°/sec)
const float MAX_JOINT_SPEED[6] = {20.0f, 15.0f, 23.0f, 34.0f, 90.0f, 90.0f};

// Posa di riposo sicura di default
const float DEFAULT_REST_POSE[6] = {0.0f, 47.0f, 0.0f, -69.0f, 7.0f, -68.0f};

// =========================================================================
// 3. PARAMETRI GEOMETRIA STRUTTURALE BRACCIO (mm)
// =========================================================================
struct RobotGeometry {
  float d12 = 50.0f;  // Altezza base -> asse J2
  float L1  = 220.0f; // Lunghezza primo braccio
  float d34 = 50.0f;  // Distanziatore roll J3 -> pitch J4
  float L2  = 220.0f; // Lunghezza avambraccio
  float d56 = 50.0f;  // Distanziatore roll J5 -> pitch J6
  float Lg  = 150.0f; // Lunghezza polso -> punta pinza
} geo;

// Costanti matematiche
const float DEG2RAD = 0.01745329251994329577f;
const float RAD2DEG = 57.2957795130823208768f;

// Stato posa attuale
float currentQ[6] = {0.0f, 47.0f, 0.0f, -69.0f, 7.0f, -68.0f};
float targetQ[6]  = {0.0f, 47.0f, 0.0f, -69.0f, 7.0f, -68.0f};
bool servosAttached = false;

// =========================================================================
// 4. FUNZIONI CINEMATICA DIRETTA (FK) & MATRICI 3x3
// =========================================================================
float clampf(float v, float a, float b){ return v < a ? a : (v > b ? b : v); }

void matMul(const float A[3][3], const float B[3][3], float C[3][3]){
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      C[i][j] = 0;
      for(int k=0; k<3; k++) C[i][j] += A[i][k] * B[k][j];
    }
  }
}

void matVecMul(const float A[3][3], const float v[3], float r[3]){
  for(int i=0; i<3; i++) r[i] = A[i][0]*v[0] + A[i][1]*v[1] + A[i][2]*v[2];
}

void fk(const float qDeg[6], float endPos[3], float endAxis[3]){
  float M[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
  float T[3][3];
  float p[3] = {0.0f, 0.0f, 0.0f};
  const float linkLengths[6] = {geo.d12, geo.L1, geo.d34, geo.L2, geo.d56, geo.Lg};
  
  for(int i=0; i<6; i++){
    float rad = qDeg[i] * DEG2RAD;
    float c = cosf(rad), s = sinf(rad);
    float Q[3][3];
    if(i % 2 == 0){
      // Giunto dispari (J1, J3, J5): Rotazione intorno asse Z
      Q[0][0] = c;  Q[0][1] = -s; Q[0][2] = 0;
      Q[1][0] = s;  Q[1][1] = c;  Q[1][2] = 0;
      Q[2][0] = 0;  Q[2][1] = 0;  Q[2][2] = 1;
    } else {
      // Giunto pari (J2, J4, J6): Rotazione intorno asse Y (Pitch)
      Q[0][0] = c;  Q[0][1] = 0;  Q[0][2] = s;
      Q[1][0] = 0;  Q[1][1] = 1;  Q[1][2] = 0;
      Q[2][0] = -s; Q[2][1] = 0;  Q[2][2] = c;
    }
    matMul(M, Q, T);
    for(int r=0; r<3; r++) for(int c2=0; c2<3; c2++) M[r][c2] = T[r][c2];
    
    float zLink[3] = {0, 0, linkLengths[i]};
    float vTrans[3];
    matVecMul(M, zLink, vTrans);
    for(int k=0; k<3; k++) p[k] += vTrans[k];
  }
  
  for(int k=0; k<3; k++){
    endPos[k] = p[k];
    endAxis[k] = M[k][2];
  }
}

// =========================================================================
// 5. RISOLUTORE LINEARE GAUSS-JORDAN (6x6)
// =========================================================================
bool solveLinear6x6(float A[6][6], float b[6], float x[6]){
  for(int k=0; k<6; k++){
    int pivot = k;
    for(int i=k+1; i<6; i++){
      if(fabsf(A[i][k]) > fabsf(A[pivot][k])) pivot = i;
    }
    if(fabsf(A[pivot][k]) < 1e-9f) return false;
    
    for(int j=k; j<6; j++){ float t = A[k][j]; A[k][j] = A[pivot][j]; A[pivot][j] = t; }
    float tb = b[k]; b[k] = b[pivot]; b[pivot] = tb;
    
    float div = A[k][k];
    for(int j=k; j<6; j++) A[k][j] /= div;
    b[k] /= div;
    
    for(int i=0; i<6; i++){
      if(i != k){
        float factor = A[i][k];
        for(int j=k; j<6; j++) A[i][j] -= factor * A[k][j];
        b[i] -= factor * b[k];
      }
    }
  }
  for(int i=0; i<6; i++) x[i] = b[i];
  return true;
}

// =========================================================================
// 6. CINEMATICA INVERSA 6 DOF IBRIDA (Analitica + DLS)
// =========================================================================
bool solveAnalyticalPlanar3Link(float rw, float zw, float L1, float L2, float L3, float gripPitchDeg, float &j2, float &j4, float &j6){
  float pitchRad = gripPitchDeg * DEG2RAD;
  float dr_grip = L3 * cosf(pitchRad);
  float dz_grip = -L3 * sinf(pitchRad);

  float r_wrist = rw - dr_grip;
  float z_wrist = zw - dz_grip;

  float dr = r_wrist;
  float dz = z_wrist - geo.d12;
  float D2 = dr * dr + dz * dz;
  float D_len = sqrtf(D2);

  if(D_len > (L1 + L2) || D_len < fabsf(L1 - L2) || D_len < 1.0f) return false;

  float cosT4 = (D2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  cosT4 = clampf(cosT4, -0.9999f, 0.9999f);
  float theta4_up = -acosf(cosT4) * RAD2DEG;

  float psi = atan2f(dz, dr) * RAD2DEG;
  float cosBeta = (L1 * L1 + D2 - L2 * L2) / (2.0f * L1 * D_len);
  cosBeta = clampf(cosBeta, -0.9999f, 0.9999f);
  float beta = acosf(cosBeta) * RAD2DEG;

  j2 = 90.0f - (psi + beta);
  j4 = theta4_up;
  j6 = 90.0f - (j2 + j4) - gripPitchDeg;
  return true;
}

bool solveIKOneSeed(const float target[3], const float seed[6], bool horizontal, float outQ[6], float &posErrMm, float &orientErr){
  for(int i=0; i<6; i++) outQ[i] = seed[i];
  
  float rxy = hypotf(target[0], target[1]);
  float ux = rxy > 1e-4f ? target[0] / rxy : 1.0f;
  float uy = rxy > 1e-4f ? target[1] / rxy : 0.0f;
  float r3d = sqrtf(target[0]*target[0] + target[1]*target[1] + target[2]*target[2]);
  
  float targetAxis[3] = {
    horizontal ? ux : (r3d > 1e-4f ? target[0]/r3d : ux),
    horizontal ? uy : (r3d > 1e-4f ? target[1]/r3d : uy),
    horizontal ? 0.0f : (r3d > 1e-4f ? target[2]/r3d : 0.0f)
  };
  
  float lambda = 0.25f;
  for(int it=0; it<250; it++){
    float endPos[3], endAxis[3];
    fk(outQ, endPos, endAxis);
    
    float dx = target[0] - endPos[0];
    float dy = target[1] - endPos[1];
    float dz = target[2] - endPos[2];
    posErrMm = sqrtf(dx*dx + dy*dy + dz*dz);
    
    float ax = targetAxis[0] - endAxis[0];
    float ay = targetAxis[1] - endAxis[1];
    float az = targetAxis[2] - endAxis[2];
    orientErr = sqrtf(ax*ax + ay*ay + az*az);
    
    if(posErrMm < 0.7f && (!horizontal || orientErr < 0.045f)) return true;
    
    float wRot = horizontal ? 32.0f : 1.0f;
    float e[6] = {dx, dy, dz, ax * wRot, ay * wRot, az * wRot};
    
    float J[6][6] = {0};
    const float delta = 0.02f;
    for(int k=0; k<6; k++){
      float qTest[6];
      for(int j=0; j<6; j++) qTest[j] = outQ[j];
      qTest[k] += delta;
      float ep[3], ap[3];
      fk(qTest, ep, ap);
      for(int r=0; r<3; r++) J[r][k] = (ep[r] - endPos[r]) / delta;
      for(int r=0; r<3; r++) J[r+3][k] = ((ap[r] - endAxis[r]) / delta) * wRot;
    }
    
    float A[6][6] = {0}, rhs[6] = {0};
    for(int i=0; i<6; i++){
      for(int j=0; j<6; j++){
        for(int r=0; r<6; r++) A[i][j] += J[r][i] * J[r][j];
        if(i == j) A[i][j] += lambda;
      }
      for(int r=0; r<6; r++) rhs[i] += J[r][i] * e[r];
    }
    
    float dq[6];
    if(!solveLinear6x6(A, rhs, dq)){
      lambda = fminf(50.0f, lambda * 2.0f);
      continue;
    }
    
    for(int k=0; k<6; k++){
      float step = clampf(dq[k], -6.0f, 6.0f);
      outQ[k] = clampf(outQ[k] + step, Q_MIN[k], Q_MAX[k]);
    }
  }
  return (posErrMm < 0.9f && (!horizontal || orientErr < 0.06f));
}

bool solveIK(float targetX, float targetY, float targetZ, bool horizontal, float solutionQ[6], float &finalErrMm){
  float target[3] = {targetX, targetY, targetZ};
  float yaw = atan2f(targetY, targetX) * RAD2DEG;
  float rxy = hypotf(targetX, targetY);
  float L1 = geo.L1 + geo.d34 * 0.5f;
  float L2 = geo.L2 + geo.d56 * 0.5f;
  float L3 = geo.Lg;
  
  float seeds[12][6];
  int seedCount = 0;
  
  // 1. Semi Analitici
  float aj2, aj4, aj6;
  if(solveAnalyticalPlanar3Link(rxy, targetZ, L1, L2, L3, 0.0f, aj2, aj4, aj6)){
    seeds[seedCount][0] = clampf(yaw, Q_MIN[0], Q_MAX[0]);
    seeds[seedCount][1] = clampf(aj2, Q_MIN[1], Q_MAX[1]);
    seeds[seedCount][2] = 0.0f;
    seeds[seedCount][3] = clampf(aj4, Q_MIN[3], Q_MAX[3]);
    seeds[seedCount][4] = 0.0f;
    seeds[seedCount][5] = clampf(aj6, Q_MIN[5], Q_MAX[5]);
    seedCount++;
  }
  
  // 2. Semi euristici Crane a minimo sforzo per la spalla J2
  float cranePresets[5][6] = {
    {yaw, 15.0f, 0.0f, -90.0f, 0.0f, 75.0f},
    {yaw, 30.0f, 0.0f, -95.0f, 0.0f, 65.0f},
    {yaw, 45.0f, 0.0f, -105.0f, 0.0f, 60.0f},
    {yaw, 10.0f, 45.0f, -90.0f, -45.0f, 75.0f},
    {0.0f, 47.0f, 0.0f, -69.0f, 7.0f, -68.0f}
  };
  for(int s=0; s<5 && seedCount<12; s++){
    for(int j=0; j<6; j++) seeds[seedCount][j] = cranePresets[s][j];
    seedCount++;
  }
  
  float bestScore = 1e9f;
  bool found = false;
  
  for(int s=0; s<seedCount; s++){
    float qSol[6], pErr, oErr;
    bool ok = solveIKOneSeed(target, seeds[s], horizontal, qSol, pErr, oErr);
    if(ok){
      float score = pErr * 50.0f + oErr * 25.0f + fabsf(qSol[1]) * 40.0f; // Penalizza sforzo J2
      if(score < bestScore){
        bestScore = score;
        finalErrMm = pErr;
        for(int j=0; j<6; j++) solutionQ[j] = qSol[j];
        found = true;
      }
    }
  }
  return found;
}

// =========================================================================
// 7. PIANIFICATORE TRAIETTORIA CON PROTEZIONE SOVRACCARICO (Anti-Stall)
// =========================================================================
void applyServoAngle(int jointIndex, float deg){
  float degClamped = clampf(deg + jointAngleOffsets[jointIndex], Q_MIN[jointIndex], Q_MAX[jointIndex]);
  
  // Mappa l'angolo in microsecondi PWM
  float spanDeg = Q_MAX[jointIndex] - Q_MIN[jointIndex];
  float norm = (degClamped - Q_MIN[jointIndex]) / spanDeg;
  int us = (int)(PWM_MIN_US[jointIndex] + norm * (PWM_MAX_US[jointIndex] - PWM_MIN_US[jointIndex]));
  
#if USE_DIRECT_ESP32_GPIO
  if(servosAttached) servos[jointIndex].writeMicroseconds(us);
#else
  // PCA9685 50Hz (Periodo 20000us, 4096 ticks)
  int ticks = (int)((us / 20000.0f) * 4096.0f);
  pwm.setPWM(servoChannels[jointIndex], 0, ticks);
#endif
}

void moveToPoseSmooth(const float startQ[6], const float goalQ[6], float speedFactor = 1.0f){
  // Calcola durata movimento in base al delta massimo
  float maxDurationMs = 300.0f;
  for(int i=0; i<6; i++){
    float d = fabsf(goalQ[i] - startQ[i]);
    float effSpeed = fmaxf(1.0f, MAX_JOINT_SPEED[i] * speedFactor);
    float tMs = (d / effSpeed) * 1000.0f;
    if(tMs > maxDurationMs) maxDurationMs = tMs;
  }
  
  const int stepDelayMs = 20;
  int totalSteps = (int)(maxDurationMs / stepDelayMs);
  if(totalSteps < 10) totalSteps = 10;
  
  for(int step=0; step<=totalSteps; step++){
    float progress = (float)step / (float)totalSteps;
    // Curva di accelerazione/decelerazione cubica (S-Curve)
    float s = progress < 0.5f ? 4.0f * progress * progress * progress : 1.0f - powf(-2.0f * progress + 2.0f, 3.0f) / 2.0f;
    
    for(int j=0; j<6; j++){
      currentQ[j] = startQ[j] + (goalQ[j] - startQ[j]) * s;
      applyServoAngle(j, currentQ[j]);
    }
    delay(stepDelayMs);
  }
}

// Esegue il movimento verificando se serve un Waypoint di Sicurezza a braccio ripiegato
void moveToPoseProtected(const float goalQ[6]){
  // Verifica se la traiettoria diretta allunga troppo la spalla (J2 > 30° e J4 > -45°)
  bool needSafeTransit = false;
  for(float s = 0.2f; s <= 0.8f; s += 0.2f){
    float testJ2 = currentQ[1] + (goalQ[1] - currentQ[1]) * s;
    float testJ4 = currentQ[3] + (goalQ[3] - currentQ[3]) * s;
    if(testJ2 > 35.0f && testJ4 > -50.0f){
      needSafeTransit = true;
      break;
    }
  }
  
  if(needSafeTransit){
    Serial.println(F("[SAFETY] Inserito Waypoint di Transito Sicuro per evitare sovraccarico motori!"));
    float transitQ[6];
    transitQ[0] = (currentQ[0] + goalQ[0]) * 0.5f;
    transitQ[1] = clampf((currentQ[1] + goalQ[1]) * 0.5f, -10.0f, 25.0f); // Spalla alzata
    transitQ[2] = (currentQ[2] + goalQ[2]) * 0.5f;
    transitQ[3] = fminf(fminf(currentQ[3], goalQ[3]), -85.0f);             // Gomito chiuso a compasso
    transitQ[4] = (currentQ[4] + goalQ[4]) * 0.5f;
    transitQ[5] = clampf(90.0f - (transitQ[1] + transitQ[3]), Q_MIN[5], Q_MAX[5]);
    
    moveToPoseSmooth(currentQ, transitQ, 1.2f);
    moveToPoseSmooth(transitQ, goalQ, 1.0f);
  } else {
    moveToPoseSmooth(currentQ, goalQ, 1.0f);
  }
  
  for(int j=0; j<6; j++) currentQ[j] = goalQ[j];
}

// =========================================================================
// 8. COMANDI SERIALI & SETUP
// =========================================================================
void attachAllServos(){
#if USE_DIRECT_ESP32_GPIO
  for(int i=0; i<6; i++){
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], PWM_MIN_US[i], PWM_MAX_US[i]);
  }
#endif
  servosAttached = true;
  Serial.println(F("[OK] Servomotori collegati e alimentati."));
}

void detachAllServos(){
#if USE_DIRECT_ESP32_GPIO
  for(int i=0; i<6; i++) servos[i].detach();
#endif
  servosAttached = false;
  Serial.println(F("[OK] Servomotori disattivati."));
}

void printStatus(){
  float endPos[3], endAxis[3];
  fk(currentQ, endPos, endAxis);
  
  Serial.println(F("=== STATO ROBOT 6 DOF ==="));
  Serial.printf("Angoli [deg]: J1=%.1f  J2=%.1f  J3=%.1f  J4=%.1f  J5=%.1f  J6=%.1f\n",
    currentQ[0], currentQ[1], currentQ[2], currentQ[3], currentQ[4], currentQ[5]);
  Serial.printf("Punta Pinza XYZ [mm]: X=%.1f  Y=%.1f  Z=%.1f  (Raggio=%.1f mm)\n",
    endPos[0], endPos[1], endPos[2], hypotf(endPos[0], endPos[1]));
  Serial.printf("Inclinazione Asse Pinza Z: %.3f (%s)\n",
    endAxis[2], fabsf(endAxis[2]) < 0.05f ? "Orizzontale Livellata" : "Inclinata");
}

void printHelp(){
  Serial.println(F("\n--- COMANDI DISPONIBILI ---"));
  Serial.println(F("1. IK <X> <Y> <Z> [0|1]  -> Risolve IK e muove la pinza alle coordinate (1 = Orizzontale)"));
  Serial.println(F("2. MOVE <J1> <J2> <J3> <J4> <J5> <J6> -> Muove i 6 giunti in gradi"));
  Serial.println(F("3. REST                  -> Ritorna alla posa di riposo ottimizzata"));
  Serial.println(F("4. STATUS                -> Stampa angoli, coordinate XYZ e orientamento"));
  Serial.println(F("5. ATTACH / DETACH       -> Attiva / Disattiva i segnali PWM dei servi"));
  Serial.println(F("6. HELP                  -> Mostra questa guida"));
}

void handleSerialCommand(String cmd){
  cmd.trim();
  if(cmd.length() == 0) return;
  
  if(cmd.equalsIgnoreCase("HELP") || cmd.equalsIgnoreCase("?")){
    printHelp();
  }
  else if(cmd.equalsIgnoreCase("STATUS") || cmd.equalsIgnoreCase("POS")){
    printStatus();
  }
  else if(cmd.equalsIgnoreCase("REST")){
    Serial.println(F("[CMD] Ritorno in posa Rest..."));
    moveToPoseProtected(DEFAULT_REST_POSE);
    printStatus();
  }
  else if(cmd.equalsIgnoreCase("ATTACH")){
    attachAllServos();
  }
  else if(cmd.equalsIgnoreCase("DETACH")){
    detachAllServos();
  }
  else if(cmd.startsWith("IK") || cmd.startsWith("ik")){
    // Formato: IK X Y Z [horizontal=1]
    float tx, ty, tz;
    int horiz = 1;
    int parsed = sscanf(cmd.c_str() + 2, "%f %f %f %d", &tx, &ty, &tz, &horiz);
    if(parsed >= 3){
      Serial.printf("[IK] Calcolo per Target XYZ: (%.1f, %.1f, %.1f) - Pinza %s...\n",
        tx, ty, tz, horiz ? "Orizzontale" : "Libera");
      float solQ[6], errMm;
      if(solveIK(tx, ty, tz, horiz != 0, solQ, errMm)){
        Serial.printf("[IK OK] Soluzione trovata! Errore pos: %.2f mm\n", errMm);
        Serial.printf("Giunti: J1=%.1f  J2=%.1f  J3=%.1f  J4=%.1f  J5=%.1f  J6=%.1f\n",
          solQ[0], solQ[1], solQ[2], solQ[3], solQ[4], solQ[5]);
        moveToPoseProtected(solQ);
        printStatus();
      } else {
        Serial.println(F("[IK ERRORE] Target fuori portata cinematica o limiti fisici!"));
      }
    } else {
      Serial.println(F("[ERRORE SINTASSI] Uso corretto: IK <X> <Y> <Z> [0|1]"));
    }
  }
  else if(cmd.startsWith("MOVE") || cmd.startsWith("move")){
    // Formato: MOVE J1 J2 J3 J4 J5 J6
    float qIn[6];
    int parsed = sscanf(cmd.c_str() + 4, "%f %f %f %f %f %f",
      &qIn[0], &qIn[1], &qIn[2], &qIn[3], &qIn[4], &qIn[5]);
    if(parsed == 6){
      Serial.println(F("[MOVE] Esecuzione posa giunti con protezione dinamica..."));
      moveToPoseProtected(qIn);
      printStatus();
    } else {
      Serial.println(F("[ERRORE SINTASSI] Uso corretto: MOVE <J1> <J2> <J3> <J4> <J5> <J6>"));
    }
  }
  else {
    Serial.println(F("[COMANDO SCONOSCIUTO] Digita HELP per visualizzare i comandi supportati."));
  }
}

// =========================================================================
// 9. SETUP & LOOP PRINCIPALE
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n=================================================="));
  Serial.println(F("   ROBOT 6 DOF - ESP32 FIRMWARE AVVIATO"));
  Serial.println(F("   Marco Codici 6 DOF · Workspace Studio"));
  Serial.println(F("=================================================="));

#if !USE_DIRECT_ESP32_GPIO
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
#endif

  attachAllServos();
  
  // Posizionamento iniziale in posa Rest protetta
  Serial.println(F("[INIT] Raggiungimento posa iniziale sicura..."));
  for(int j=0; j<6; j++){
    currentQ[j] = DEFAULT_REST_POSE[j];
    applyServoAngle(j, currentQ[j]);
  }
  
  printHelp();
  printStatus();
}

void loop() {
  if(Serial.available() > 0){
    String cmd = Serial.readStringUntil('\n');
    handleSerialCommand(cmd);
  }
  delay(10);
}
