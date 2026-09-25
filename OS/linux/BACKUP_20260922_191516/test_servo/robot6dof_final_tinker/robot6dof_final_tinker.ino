#include <ESP32Servo.h>

// =====================================================
// ESP32 - ROBOT 6DOF FINAL + TINKERBOARD BRIDGE
// =====================================================
// MAPPATURA FINALE CORRETTA (da test_servo_finale.ino):
// J1 -> GPIO 23   (dispari, 270°, 500-2450µs)
// J2 -> GPIO 19   (pari,  180°, 500-1800µs) + GPIO 14 INVERSO
// J3 -> GPIO 25   (dispari, 270°, 500-2450µs)
// J4 -> GPIO 26   (pari,  180°, 500-1800µs)
// J5 -> GPIO 18   (dispari, 270°, 500-2450µs)
// J6 -> GPIO 13   (pari,  180°, 500-1800µs)
//
// PROTOCOLLO TINKERBOARD (da robot6dof-v2):
// Tinkerboard invia su Serial (USB) o Serial2 (UART):
//   SEG <id> <dur_ms> <q1..q6>   // q in coordinate Tinker: J1,3,5 -135..135, J2,4,6 -90..90
//   ARM <id> <q1..q6>
//   STOP <id>
//   PING <id>
// Risponde JSON su stessa porta + broadcast:
//   {"device":"robot6dof","protocol":1,"type":"state","q":[..],"armed":true/false}
//   {"device":"robot6dof","protocol":1,"type":"done","id":"..."}
//   {"device":"robot6dof","protocol":1,"type":"armed","id":"..."}
//   {"device":"robot6dof","protocol":1,"type":"error","id":"...","error":"..."}
// Manuale locale (debug via USB):
//   j1 90g , j2 45g ecc.  // angoli HW 0-270 pari/dispari
// =====================================================

// --- PIN ---
const int SERVO_PIN_J1 = 23;
const int SERVO_PIN_J2 = 19;
const int SERVO_PIN_J2_INV = 14;
const int SERVO_PIN_J3 = 25;
const int SERVO_PIN_J4 = 26;
const int SERVO_PIN_J5 = 18;
const int SERVO_PIN_J6 = 13;

// UART per TinkerBoard: Serial2 = GPIO16 RX2 / GPIO17 TX2 @115200
// Se colleghi via USB, usa Serial (USB CDC) - il firmware ascolta entrambi.
// Se vuoi usare altri pin, cambia qui e ricollega i fili.
#define TINKER_BAUD 115200
#define TINKER_RX 16
#define TINKER_TX 17

// --- SERVI ---
Servo servoJ1, servoJ2, servoJ2Inv, servoJ3, servoJ4, servoJ5, servoJ6;

// --- PWM ---
const int PWM_MIN_ODD  = 500;
const int PWM_MAX_ODD  = 2450;
const int PWM_MIN_EVEN = 500;
const int PWM_MAX_EVEN = 1800;

// --- VELOCITA' (da test_servo_finale) ---
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
float PASSO_MULTIPLIER;
int LOOP_DELAY;
float RAMPA_EXP;

// --- POSIZIONI HW (0..270 odd, 0..180 even) ---
float posizioneJ1 = 135;
float posizioneJ2 = 90;
float posizioneJ3 = 135;
float posizioneJ4 = 90;
float posizioneJ5 = 135;
float posizioneJ6 = 90;

// --- STATO TINKER ---
static const char* DEVICE = "robot6dof";
static const int PROTOCOL = 1;
bool armed = false;
bool moving = false;
unsigned long lastStateMs = 0;
#define STATE_INTERVAL 100

struct PendingRequest {
  String id;
  String type;
  unsigned long timeoutMs = 0;
  bool active = false;
};
PendingRequest pending;

// =====================================================
// UTIL
// =====================================================
float getMaxAngle(int n) { return (n % 2 == 1) ? 270.0 : 180.0; }

// Tinker -> HW: tinker -135..135 (odd) => 0..270 ; -90..90 (even) => 0..180
// FIX J2 direzione invertita (specchiato) - coincide con simulazione software
float tinkerToHw(int j, float tinkerAngle) {
  if (j == 2) return constrain(90.0 - tinkerAngle, 0, 180); // J2 invertito
  if (j % 2 == 1) return constrain(tinkerAngle + 135.0, 0, 270);
  else return constrain(tinkerAngle + 90.0, 0, 180);
}
float hwToTinker(int j, float hwAngle) {
  if (j == 2) return 90.0 - hwAngle; // J2 invertito
  if (j % 2 == 1) return hwAngle - 135.0;
  else return hwAngle - 90.0;
}

float smoothstep(float t) {
  t = constrain(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

int angleToUs(int n, float angle) {
  if (n % 2 == 1) {
    angle = constrain(angle, 0, 270);
    return PWM_MIN_ODD + (angle / 270.0) * (PWM_MAX_ODD - PWM_MIN_ODD);
  } else {
    angle = constrain(angle, 0, 180);
    return PWM_MIN_EVEN + (angle / 180.0) * (PWM_MAX_EVEN - PWM_MIN_EVEN);
  }
}
int angleToUsInv(float angle) {
  angle = constrain(angle, 0, 180);
  return PWM_MIN_EVEN + ((180.0 - angle) / 180.0) * (PWM_MAX_EVEN - PWM_MIN_EVEN);
}

void setHwServo(int idx, float hwAngle) {
  // idx 0..5
  if (idx == 1) { // J2 doppia
    posizioneJ2 = hwAngle;
    servoJ2.writeMicroseconds(angleToUs(2, hwAngle));
    servoJ2Inv.writeMicroseconds(angleToUsInv(hwAngle));
  } else if (idx == 0) { posizioneJ1 = hwAngle; servoJ1.writeMicroseconds(angleToUs(1, hwAngle)); }
  else if (idx == 2) { posizioneJ3 = hwAngle; servoJ3.writeMicroseconds(angleToUs(3, hwAngle)); }
  else if (idx == 3) { posizioneJ4 = hwAngle; servoJ4.writeMicroseconds(angleToUs(4, hwAngle)); }
  else if (idx == 4) { posizioneJ5 = hwAngle; servoJ5.writeMicroseconds(angleToUs(5, hwAngle)); }
  else if (idx == 5) { posizioneJ6 = hwAngle; servoJ6.writeMicroseconds(angleToUs(6, hwAngle)); }
}
float getHwPos(int idx) {
  if (idx==0) return posizioneJ1;
  if (idx==1) return posizioneJ2;
  if (idx==2) return posizioneJ3;
  if (idx==3) return posizioneJ4;
  if (idx==4) return posizioneJ5;
  if (idx==5) return posizioneJ6;
  return 0;
}

// Invio JSON su entrambe le seriali (Serial + Serial2) per coprire USB e UART
void sendJson(const String& json) {
  Serial.println(json);
  Serial2.println(json);
}
void sendState() {
  String q = "[";
  q += String(hwToTinker(1, posizioneJ1),2) + ", ";
  q += String(hwToTinker(2, posizioneJ2),2) + ", ";
  q += String(hwToTinker(3, posizioneJ3),2) + ", ";
  q += String(hwToTinker(4, posizioneJ4),2) + ", ";
  q += String(hwToTinker(5, posizioneJ5),2) + ", ";
  q += String(hwToTinker(6, posizioneJ6),2) + "]";
  String json = "{\"device\":\"" + String(DEVICE) + "\",\"protocol\":" + String(PROTOCOL) + ",\"type\":\"state\",\"q\":" + q + ",\"armed\":" + (armed ? "true" : "false") + "}";
  sendJson(json);
}
void sendDone(const String& id) { sendJson("{\"device\":\"" + String(DEVICE) + "\",\"protocol\":" + String(PROTOCOL) + ",\"type\":\"done\",\"id\":\"" + id + "\"}"); }
void sendArmed(const String& id) { sendJson("{\"device\":\"" + String(DEVICE) + "\",\"protocol\":" + String(PROTOCOL) + ",\"type\":\"armed\",\"id\":\"" + id + "\"}"); }
void sendStopped(const String& id) { sendJson("{\"device\":\"" + String(DEVICE) + "\",\"protocol\":" + String(PROTOCOL) + ",\"type\":\"stopped\",\"id\":\"" + id + "\"}"); }
void sendError(const String& id, const String& msg) { sendJson("{\"device\":\"" + String(DEVICE) + "\",\"protocol\":" + String(PROTOCOL) + ",\"type\":\"error\",\"id\":\"" + id + "\",\"error\":\"" + msg + "\"}"); }

// =====================================================
// MOVIMENTO FLUIDO (S-CURVE) - da test_servo_finale
// =====================================================
void muoviServoFluido(Servo &servo, float &posizione, float destinazione, int numeroServo) {
  float maxAngle = getMaxAngle(numeroServo);
  destinazione = constrain(destinazione, 0, maxAngle);
  float start = posizione;
  float totalDist = destinazione - start;
  float distanzaTotale = abs(totalDist);
  if (distanzaTotale < 0.5) { posizione = destinazione; servo.writeMicroseconds(angleToUs(numeroServo, posizione)); return; }
  float direzione = (totalDist > 0) ? 1.0 : -1.0;
  while (true) {
    float distanzaPercorsa = abs(posizione - start);
    float progresso = constrain(distanzaPercorsa / distanzaTotale, 0.0, 1.0);
    float vel = SMOOTH_START + (1.0 - SMOOTH_START) * smoothstep(pow(progresso, RAMPA_EXP));
    float velocitaMax = map(VELOCITA, 0, 100, 5, 100);
    float velocita = velocitaMax * vel;
    if (velocita < 0.3) velocita = 0.3;
    float passo = velocita * PASSO_MULTIPLIER;
    float distanzaRimanente = abs(destinazione - posizione);
    if (passo > distanzaRimanente) passo = distanzaRimanente;
    posizione += direzione * passo;
    if (abs(posizione - destinazione) < 0.5) { posizione = destinazione; servo.writeMicroseconds(angleToUs(numeroServo, posizione)); break; }
    servo.writeMicroseconds(angleToUs(numeroServo, posizione));
    delay(LOOP_DELAY);
  }
}

void muoviJ2(float destinazione) {
  destinazione = constrain(destinazione, 0, 180);
  float start = posizioneJ2;
  float totalDist = destinazione - start;
  float distanzaTotale = abs(totalDist);
  if (distanzaTotale < 0.5) {
    posizioneJ2 = destinazione;
    servoJ2.writeMicroseconds(angleToUs(2, posizioneJ2));
    servoJ2Inv.writeMicroseconds(angleToUsInv(posizioneJ2));
    return;
  }
  float direzione = (totalDist > 0) ? 1.0 : -1.0;
  while (true) {
    float distanzaPercorsa = abs(posizioneJ2 - start);
    float progresso = constrain(distanzaPercorsa / distanzaTotale, 0.0, 1.0);
    float vel = SMOOTH_START + (1.0 - SMOOTH_START) * smoothstep(pow(progresso, RAMPA_EXP));
    float velocitaMax = map(VELOCITA_J2, 0, 100, 3, 70);
    float velocita = velocitaMax * vel * POTENZA_J2;
    if (posizioneJ2 < J2_SLOW_ZONE) {
      float factor = posizioneJ2 / J2_SLOW_ZONE;
      velocita *= J2_SLOW_FACTOR + factor * (1.0 - J2_SLOW_FACTOR);
    } else if (posizioneJ2 > (180.0 - J2_SLOW_ZONE)) {
      float factor = (180.0 - posizioneJ2) / J2_SLOW_ZONE;
      velocita *= J2_SLOW_FACTOR + factor * (1.0 - J2_SLOW_FACTOR);
    }
    if (velocita < 0.3) velocita = 0.3;
    float passo = velocita * PASSO_MULTIPLIER;
    float distanzaRimanente = abs(destinazione - posizioneJ2);
    if (passo > distanzaRimanente) passo = distanzaRimanente;
    posizioneJ2 += direzione * passo;
    if (abs(posizioneJ2 - destinazione) < 0.5) {
      posizioneJ2 = destinazione;
      servoJ2.writeMicroseconds(angleToUs(2, posizioneJ2));
      servoJ2Inv.writeMicroseconds(angleToUsInv(posizioneJ2));
      break;
    }
    servoJ2.writeMicroseconds(angleToUs(2, posizioneJ2));
    servoJ2Inv.writeMicroseconds(angleToUsInv(posizioneJ2));
    delay(LOOP_DELAY);
  }
  posizioneJ2 = destinazione;
  servoJ2.writeMicroseconds(angleToUs(2, posizioneJ2));
  servoJ2Inv.writeMicroseconds(angleToUsInv(posizioneJ2));
}

// Wrapper per movimento singolo giunto in HW
void vaiA(Servo &servo, float &posizione, float destinazione, int numeroServo) {
  destinazione = constrain(destinazione, 0, getMaxAngle(numeroServo));
  if (numeroServo == 2) { muoviJ2(destinazione); return; }
  muoviServoFluido(servo, posizione, destinazione, numeroServo);
}

// =====================================================
// HANDLER TINKERBOARD - SEG con durata + S-CURVE
// =====================================================
void handleSeg(String args) {
  // SEG <id> <dur_ms> <q1..q6 tinker>
  args.trim();
  int p1 = args.indexOf(' ');
  if (p1==-1) { sendError("seg","formato SEG invalido"); return; }
  String id = args.substring(0,p1); args = args.substring(p1+1); args.trim();
  int p2 = args.indexOf(' ');
  if (p2==-1) { sendError(id,"durata mancante"); return; }
  int duration_ms = args.substring(0,p2).toInt(); args = args.substring(p2+1);
  float targetTinker[6]; float targetHw[6];
  for(int i=0;i<6;i++){
    args.trim();
    int sp = args.indexOf(' ');
    String tok = (sp==-1) ? args : args.substring(0,sp);
    if(sp!=-1) args = args.substring(sp+1);
    targetTinker[i] = tok.toFloat();
    // valida range tinker
    float mn = ( (i%2==0) ? -135 : -90);
    float mx = ( (i%2==0) ? 135 : 90);
    if(targetTinker[i] < mn-0.01 || targetTinker[i] > mx+0.01){
      sendError(id, "Angolo fuori limiti J"+String(i+1));
      return;
    }
    targetHw[i] = tinkerToHw(i+1, targetTinker[i]);
  }

  // FIX SCATTI: limita velocità max 30°/s e invia state durante movimento per evitare timeout 4s
  moving = true;
  pending.id = id; pending.type="SEG"; pending.active=true; pending.timeoutMs = millis() + duration_ms + 8000;
  float startHw[6];
  for(int i=0;i<6;i++) startHw[i]=getHwPos(i);
  // Calcola durata minima in base a delta max e velocità max (evita scatti)
  float maxDelta = 0;
  for(int i=0;i<6;i++) maxDelta = max(maxDelta, abs(targetHw[i]-startHw[i]));
  int minDuration = (int)(maxDelta / 30.0 * 1000); // 30°/s max
  if (duration_ms < minDuration) duration_ms = minDuration;
  if(duration_ms < 200) duration_ms = 200; // minimo 200ms per non scattare
  int steps = max(duration_ms / 20, 1);
  unsigned long lastStateSend = millis();
  for(int step=1; step<=steps; step++){
    float t = (float)step / steps;
    float ease = t < 0.5 ? 4*t*t*t : 1 - pow(-2*t+2, 3)/2;
    for(int i=0;i<6;i++){
      float hw = startHw[i] + ease * (targetHw[i] - startHw[i]);
      setHwServo(i, hw);
    }
    // invia state ogni 80ms anche durante movimento (evita timeout PC)
    if (millis() - lastStateSend > 80) { sendState(); lastStateSend = millis(); }
    delay(20);
    if(!pending.active) { moving=false; return; }
  }
  for(int i=0;i<6;i++) setHwServo(i, targetHw[i]);
  sendState();
  moving = false;
  pending.active = false;
  sendDone(id);
}

void handleArm(String args) {
  args.trim();
  // ARM <id> <q1..q6>  oppure  ARM arm <q1..q6>
  int p1 = args.indexOf(' ');
  if(p1==-1){ sendError("arm","formato ARM invalido"); return; }
  String id = args.substring(0,p1); args = args.substring(p1+1);
  float q[6];
  for(int i=0;i<6;i++){
    args.trim();
    int sp = args.indexOf(' ');
    String tok = (sp==-1) ? args : args.substring(0,sp);
    if(sp!=-1) args = args.substring(sp+1);
    q[i]=tok.toFloat();
    float mn = ( (i%2==0) ? -135 : -90);
    float mx = ( (i%2==0) ? 135 : 90);
    if(q[i] < mn-0.01 || q[i] > mx+0.01){ sendError(id,"Angolo fuori limiti J"+String(i+1)); return;}
  }
  for(int i=0;i<6;i++) setHwServo(i, tinkerToHw(i+1,q[i]));
  armed = true;
  sendArmed(id);
}
void handleStop(String args){
  args.trim();
  String id = args; if(id.length()==0) id="stop";
  pending.active=false;
  moving=false;
  sendStopped(id);
}
void handlePing(String args){
  args.trim();
  String id = args; if(id.length()==0) id="ping";
  sendJson("{\"device\":\""+String(DEVICE)+"\",\"protocol\":"+String(PROTOCOL)+",\"type\":\"pong\",\"id\":\""+id+"\"}");
}

// Calibrazione singola giunti: centrale +-10 con pause 5s
void handleCal(String args){
  args.trim(); // args: id opzionale
  String id = args.length()? args : "cal_"+String(millis());
  sendJson("{\"device\":\""+String(DEVICE)+"\",\"protocol\":"+String(PROTOCOL)+",\"type\":\"cal_start\",\"id\":\""+id+"\"}");
  for(int j=1;j<=6;j++){
    float center = (j%2==1)?135:90;
    float plus = center + 10;
    float minus = center - 10;
    if(j%2==0){ plus = constrain(plus,0,180); minus = constrain(minus,0,180); }
    else { plus = constrain(plus,0,270); minus = constrain(minus,0,270); }
    // centrale
    if(j==2) muoviJ2(center); else { Servo *s=nullptr; float *p=nullptr; if(j==1){s=&servoJ1;p=&posizioneJ1;}else if(j==3){s=&servoJ3;p=&posizioneJ3;}else if(j==4){s=&servoJ4;p=&posizioneJ4;}else if(j==5){s=&servoJ5;p=&posizioneJ5;}else if(j==6){s=&servoJ6;p=&posizioneJ6;} muoviServoFluido(*s,*p,center,j); }
    sendJson("{\"device\":\""+String(DEVICE)+"\",\"protocol\":"+String(PROTOCOL)+",\"type\":\"cal_step\",\"id\":\""+id+"\",\"joint\":"+String(j)+",\"pos\":\"center\"}");
    delay(5000);
    // +10
    if(j==2) muoviJ2(plus); else { Servo *s=nullptr; float *p=nullptr; if(j==1){s=&servoJ1;p=&posizioneJ1;}else if(j==3){s=&servoJ3;p=&posizioneJ3;}else if(j==4){s=&servoJ4;p=&posizioneJ4;}else if(j==5){s=&servoJ5;p=&posizioneJ5;}else if(j==6){s=&servoJ6;p=&posizioneJ6;} muoviServoFluido(*s,*p,plus,j); }
    sendJson("{\"device\":\""+String(DEVICE)+"\",\"protocol\":"+String(PROTOCOL)+",\"type\":\"cal_step\",\"id\":\""+id+"\",\"joint\":"+String(j)+",\"pos\":\"plus10\"}");
    delay(5000);
    // -10
    if(j==2) muoviJ2(minus); else { Servo *s=nullptr; float *p=nullptr; if(j==1){s=&servoJ1;p=&posizioneJ1;}else if(j==3){s=&servoJ3;p=&posizioneJ3;}else if(j==4){s=&servoJ4;p=&posizioneJ4;}else if(j==5){s=&servoJ5;p=&posizioneJ5;}else if(j==6){s=&servoJ6;p=&posizioneJ6;} muoviServoFluido(*s,*p,minus,j); }
    sendJson("{\"device\":\""+String(DEVICE)+"\",\"protocol\":"+String(PROTOCOL)+",\"type\":\"cal_step\",\"id\":\""+id+"\",\"joint\":"+String(j)+",\"pos\":\"minus10\"}");
    delay(5000);
    // torna centrale
    if(j==2) muoviJ2(center); else { Servo *s=nullptr; float *p=nullptr; if(j==1){s=&servoJ1;p=&posizioneJ1;}else if(j==3){s=&servoJ3;p=&posizioneJ3;}else if(j==4){s=&servoJ4;p=&posizioneJ4;}else if(j==5){s=&servoJ5;p=&posizioneJ5;}else if(j==6){s=&servoJ6;p=&posizioneJ6;} muoviServoFluido(*s,*p,center,j); }
    sendJson("{\"device\":\""+String(DEVICE)+"\",\"protocol\":"+String(PROTOCOL)+",\"type\":\"cal_step\",\"id\":\""+id+"\",\"joint\":"+String(j)+",\"pos\":\"center_end\"}");
    delay(2000);
  }
  sendJson("{\"device\":\""+String(DEVICE)+"\",\"protocol\":"+String(PROTOCOL)+",\"type\":\"cal_done\",\"id\":\""+id+"\"}");
  sendState();
}
void handleTinkerLine(String line){
  line.trim();
  if(line.length()==0) return;
  if(line.startsWith("SEG ")) handleSeg(line.substring(4));
  else if(line.startsWith("ARM ")) handleArm(line.substring(4));
  else if(line.startsWith("STOP")) handleStop(line.substring(4));
  else if(line.startsWith("PING")) handlePing(line.substring(4));
  else if(line.startsWith("CAL")) handleCal(line.substring(3));
  else if(line.startsWith("seg ")) handleSeg(line.substring(4));
  else {
  }
}

// =====================================================
// COMANDI MANUALI jX (da test_servo_finale)
// =====================================================
void eseguiComando(int numeroServo, String comando);
void vaiAManual(Servo &servo, float &posizione, float destinazione, int numeroServo){
  if(numeroServo==2) muoviJ2(destinazione);
  else muoviServoFluido(servo, posizione, destinazione, numeroServo);
}
void eseguiComando(int numeroServo, String comando){
  Servo *servo=nullptr; float *posizione=nullptr;
  switch(numeroServo){
    case 1: servo=&servoJ1; posizione=&posizioneJ1; break;
    case 2: servo=&servoJ2; posizione=&posizioneJ2; break;
    case 3: servo=&servoJ3; posizione=&posizioneJ3; break;
    case 4: servo=&servoJ4; posizione=&posizioneJ4; break;
    case 5: servo=&servoJ5; posizione=&posizioneJ5; break;
    case 6: servo=&servoJ6; posizione=&posizioneJ6; break;
    default: Serial.println("ERRORE: servo non valido."); return;
  }
  comando.trim();
  // comandi 0-6 test
  if(comando=="0"||comando=="1"||comando=="2"||comando=="3"||comando=="4"||comando=="5"||comando=="6"){
    int nc=comando.toInt();
    bool isPari=(numeroServo%2==0);
    float centro=isPari?90:135;
    float mx=isPari?180:270;
    switch(nc){
      case 0: vaiAManual(*servo,*posizione,0,numeroServo); break;
      case 1: vaiAManual(*servo,*posizione,centro,numeroServo); break;
      case 2: vaiAManual(*servo,*posizione,mx,numeroServo); break;
      case 3: vaiAManual(*servo,*posizione,centro,numeroServo); vaiAManual(*servo,*posizione,0,numeroServo); vaiAManual(*servo,*posizione,centro,numeroServo); break;
      case 4: vaiAManual(*servo,*posizione,centro,numeroServo); vaiAManual(*servo,*posizione,mx,numeroServo); vaiAManual(*servo,*posizione,centro,numeroServo); break;
      case 5: vaiAManual(*servo,*posizione,0,numeroServo); vaiAManual(*servo,*posizione,mx,numeroServo); vaiAManual(*servo,*posizione,0,numeroServo); break;
      case 6: vaiAManual(*servo,*posizione,centro,numeroServo); vaiAManual(*servo,*posizione,0,numeroServo); vaiAManual(*servo,*posizione,mx,numeroServo); vaiAManual(*servo,*posizione,centro,numeroServo); break;
    }
    return;
  }
  if(comando.endsWith("g")){
    float ang=comando.substring(0, comando.length()-1).toFloat();
    if(ang <0 || ang>getMaxAngle(numeroServo)){ Serial.println("ERRORE: fuori range"); return; }
    vaiAManual(*servo,*posizione,ang,numeroServo);
    return;
  }
  Serial.println("ERRORE: comando non riconosciuto. Usa 0-6 o 20g");
}

bool tryHandleManual(String line){
  line.trim(); line.toLowerCase();
  if(!line.startsWith("j")) return false;
  if(line.length()<2) return false;
  int n = line.charAt(1)-'0';
  if(n<1||n>6) return false;
  int sp=line.indexOf(' ');
  if(sp==-1){ Serial.println("FORMATO: j1 90g"); return true; }
  String cmd=line.substring(sp+1); cmd.trim();
  eseguiComando(n, cmd);
  // Aggiorna armed su movimento manuale
  armed=true;
  sendState();
  return true;
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup(){
  Serial.begin(115200);
  Serial2.begin(TINKER_BAUD, SERIAL_8N1, TINKER_RX, TINKER_TX);
  delay(500);

  float master=constrain(VELOCITA_MASTER,0,100);
  float multiplier=pow(2.0,(master-50)/25.0);
  PASSO_MULTIPLIER=PASSO_MULTIPLIER_BASE*multiplier;
  LOOP_DELAY=max(1,(int)(LOOP_DELAY_BASE/multiplier));
  float rampa=constrain(VELOCITA_RAMPA,0,100);
  RAMPA_EXP=1.0+2.0*(1.0 - rampa/50.0);
  RAMPA_EXP=constrain(RAMPA_EXP,0.3,3.0);

  // FIX SCATTO INIZIALE: attach senza scatto, poi homing lento
  servoJ1.attach(SERVO_PIN_J1, PWM_MIN_ODD, PWM_MAX_ODD);
  delay(80);
  servoJ2.attach(SERVO_PIN_J2, PWM_MIN_EVEN, PWM_MAX_EVEN);
  delay(80);
  servoJ2Inv.attach(SERVO_PIN_J2_INV, PWM_MIN_EVEN, PWM_MAX_EVEN);
  delay(80);
  servoJ3.attach(SERVO_PIN_J3, PWM_MIN_ODD, PWM_MAX_ODD);
  delay(80);
  servoJ4.attach(SERVO_PIN_J4, PWM_MIN_EVEN, PWM_MAX_EVEN);
  delay(80);
  servoJ5.attach(SERVO_PIN_J5, PWM_MIN_ODD, PWM_MAX_ODD);
  delay(80);
  servoJ6.attach(SERVO_PIN_J6, PWM_MIN_EVEN, PWM_MAX_EVEN);
  delay(300);
  // Homing dolce: non forzare subito a 135/90 con scatto, ma muovi lentamente se necessario
  // Se i servo sono già vicini al centro, il movimento sarà minimo e senza scatto
  // Altrimenti muovi con S-curve lenta
  {
    float targetCenters[6] = {135,90,135,90,135,90};
    // Imposta posizione logica senza muovere bruscamente, poi muovi con S-curve solo se delta >2°
    for(int i=0;i<6;i++){
      float cur = getHwPos(i);
      float tgt = targetCenters[i];
      if (abs(tgt - cur) > 2.0) {
        // Movimento lento per homing
        float oldVel = VELOCITA; int oldMaster = VELOCITA_MASTER;
        VELOCITA = 35; // lenta per homing
        if(i==1) muoviJ2(tgt);
        else {
          Servo *s=nullptr; float *p=nullptr;
          if(i==0){s=&servoJ1;p=&posizioneJ1;}else if(i==2){s=&servoJ3;p=&posizioneJ3;}else if(i==3){s=&servoJ4;p=&posizioneJ4;}else if(i==4){s=&servoJ5;p=&posizioneJ5;}else if(i==5){s=&servoJ6;p=&posizioneJ6;}
          muoviServoFluido(*s,*p,tgt,i+1);
        }
        VELOCITA = oldVel; VELOCITA_MASTER = oldMaster;
      } else {
        setHwServo(i, tgt);
      }
    }
  }

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  ESP32 6-DOF FINAL + TINKER BRIDGE");
  Serial.println("==========================================");
  Serial.println("PIN: J1:23 J2:19+14(inv) J3:25 J4:26 J5:18 J6:13");
  Serial.println("PWM odd 500-2450 (270) even 500-1800 (180)");
  Serial.println("Tinker: Serial + Serial2(16/17) @115200");
  Serial.println("Protocollo: SEG/ARM/STOP/PING <-> JSON state");
  Serial.println("Manuale: j1 90g / j1 0..6");
  Serial.println("Tinker angles: odd -135..135 even -90..90");
  Serial.println("HW angles: odd 0..270 even 0..180");
  Serial.println("==========================================");
  Serial2.println("ESP32 TINKER BRIDGE PRONTO");
  armed=false;
}

void loop(){
  // -- Poll Serial (USB) --
  if(Serial.available()){
    String line=Serial.readStringUntil('\n');
    line.trim();
    if(line.length()){
      String low=line; low.toLowerCase();
      if(low.startsWith("seg ")||low.startsWith("arm ")||low.startsWith("stop")||low.startsWith("ping")){
        // case insensitive -> normalizza maiuscolo per handler
        line.toUpperCase(); // SEG ecc. diventa maiuscolo, id resta maiuscolo ok
        // Ma gli id sono case sensitive; manteniamo come maiuscolo va bene per tinker che genera hex maiuscolo
        handleTinkerLine(line);
      } else if(low.startsWith("j1")||low.startsWith("j2")||low.startsWith("j3")||low.startsWith("j4")||low.startsWith("j5")||low.startsWith("j6")){
        tryHandleManual(line);
      } else {
        Serial.println("Comando non riconosciuto. Usa SEG/ARM/STOP o j1 90g");
      }
    }
  }
  // -- Poll Serial2 (UART Tinker) --
  if(Serial2.available()){
    String line=Serial2.readStringUntil('\n');
    line.trim();
    if(line.length()){
      String low=line; low.toLowerCase();
      if(low.startsWith("seg ")||low.startsWith("arm ")||low.startsWith("stop")||low.startsWith("ping")){
        line.toUpperCase();
        handleTinkerLine(line);
      } else {
        // per debug, echo anche su Serial
        Serial.println("[S2] "+line);
      }
    }
  }
  // -- State broadcast --
  if(millis() - lastStateMs > STATE_INTERVAL){
    lastStateMs=millis();
    sendState();
  }
  // -- Timeout pending --
  if(pending.active && millis() > pending.timeoutMs){
    sendError(pending.id, "timeout");
    pending.active=false;
    moving=false;
  }
}
