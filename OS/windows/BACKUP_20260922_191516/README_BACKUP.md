# BACKUP PROGETTO — Robot 6 DOF / Marco-Codici-6-DOF

## Dati identificativi

| Campo | Valore |
|---|---|
| **Data backup** | 22 settembre 2026 (22.09.2026) |
| **Ora approssimativa** | 19:15 (ora locale Windows) |
| **Cartella backup** | `C:\Users\Marco Trulli\Desktop\Marco-Codici-6-DOF-main\BACKUP_20260922_191516` |
| **Dimensione totale** | ~15,6 MB (67+ file totali nelle cartelle progetto) |
| **Stato sistema al momento del backup** | Server Node raggiungibile su `http://localhost:8765`; ESP32 flashato con firmware corretto; porta COM5 funzionante |

---

## COSA CONTIENE QUESTO BACKUP (albero)

```
BACKUP_20260922_191516/
├── README_BACKUP.md                 ← QUESTO FILE
├── Marco-Codici-6-DOF-main/         ← PROGETTO LIVE (sorgenti eseguibili)
├── firmware/                        ← Firmware ESP32 (sorgente .ino + README flash)
├── foto braccio/                    ← 26 foto WhatsApp del braccio robotico REALE
├── desktop-app/                     ← App desktop Tauri (launcher) SENZA artefatti build
├── Robot6DOF_ESP32_FLASH/           ← Copia del .ino usata per il flash manuale del 22.09.26
└── test_servo/                      ← Laboratorio storico: da qui nascono pin/PWM/movimenti
```

**Cosa NON è incluso (volutamente):**
- `desktop-app/src-tauri/target/` (~1,25 GB) — artefatti di compilazione Rust/Tauri, ricostruibili con `cargo build`
- `node_modules/` — dipendenze npm, ripristinabili con `npm install` (c'è `package-lock.json`)
- Backup precedente `Marco-Codici-6-DOF-main_BACKUP_20260919_191941` — già esistente nella cartella padre

---

## 1. `Marco-Codici-6-DOF-main/` — Il progetto live

Cartella **da avviare** per usare il sistema. Contiene sorgenti aggiornate al 22.09.2026.

### 1.1 File principali

| File | Ruolo |
|---|---|
| **`server.js`** (24 KB, ~600 righe) | Backend Node.js/Express. Ascolta su **porta 8765**. Gestisce: seriale ESP32 (COM5 @115200, protocollo JSON), API REST per il simulatore, proxy Kinect, salvataggio config, single-flight riconnessione seriale. |
| **`ik_simulator_v30.html`** (247 KB, ~6500 righe) | Interfaccia web completa: simulatore 3D Three.js, cinematica inversa 6 DOF, FEA, pose/animazioni, pannello Kinect (depth/RGB/nuvola punti), comandi ESP32. **È la UI da aprire.** |
| **`Program.cs`** | Launcher C# alternativo con API ESP32 **virtuali** (simulazione, nessuna seriale reale). Utile solo per demo senza hardware. |
| **`package.json`** / `package-lock.json` | Dipendenze npm: `express`, `serialport@^12`, `cors`. |
| **`index.html`** | Pagina di boot/minima (la UI vera è `ik_simulator_v30.html`). |
| **`ISTRUZIONI_AVVIO_CORRETTO.txt`** | Istruzioni avvio: aprire `http://localhost:8765/ik_simulator_v30.html`. |
| **`README.md`** | README generico del repo (struttura, quick start, licenza MIT). |
| **`PREPARA_GITHUB.bat`** / **`.gitignore`** | Utilità git. |

### 1.2 Sottocartelle

| Cartella | Contenuto specifico |
|---|---|
| **`config/`** | JSON di runtime: pose salvate (`robot6dof_saved_poses.json`), animazioni, physics, speeds, notes, stl_config. Scritti via API `/api/save_*`. |
| **`docs/`** | Documentazione: README, QUICKSTART, DEPLOYMENT_RASPBERRY, CONTRIBUTING, system prompt. |
| **`firmware/`** | **Firmware VECCHIO** `Robot6DOF_ESP32_Firmware.ino` (protocollo testuale IK/MOVE/REST — NON usare con il server attuale). Quello buono è in `../firmware/` (cartella radice del backup). |
| **`kinect/`** | `KinectBridge.cs` (sorgente) + `KinectBridge.exe` (binario). Bridge Kinect for Windows SDK 1.8 → TCP **porta 8766**. Espone: `/status`, frame **depth**, frame **RGB**. **Nessun body tracking/skeleton.** Avviato automaticamente da `server.js` se l'exe esiste. |
| **`libs/`** | Librerie JS offline: `three.min.js`, `STLLoader.js`, `TransformControls.js` (fallback CDN se mancanti). |
| **`models/`** | Mesh STL del braccio: `braccio_1.stl`, `braccio_2.stl`, `braccio_fine.stl`, `collegamento_giunti_braccio_1/2/3.stl` (+ varianti `_vecchio`). Caricate nel visualizzatore 3D. |
| **`scripts/`** | Batch/shell: `setup`, `start`, `AVVIA_SIMULATORE`, sincronizzazione GitHub. |
| **`simulator_versions/`** | Storico UI: `ik_simulator_v2.html` … `ik_simulator_v29.html` (versioni precedenti di v30). |

### 1.3 Protocollo seriale PC ↔ ESP32 (come funziona oggi)

- **Porta**: COM5 prioritaria, auto-detect USB-serial (VID whitelist: 1A86, 10C4, 303A, …). Mai COM1/Bluetooth.
- **Baud**: 115200
- **PC → ESP32** (ASCII + `\n`): `SEG <id> <dur_ms> <q1..q6>`, `ARM <id> <q1..q6>`, `STOP <id>`, `PING <id>`, `CAL <id>`
- **ESP32 → PC** (JSON + `\n`, ~10 Hz; 80 ms durante SEG):
  ```json
  {"device":"robot6dof","protocol":1,"type":"state","q":[...6 valori tinker...],"armed":true}
  ```
  Altri tipi: `done`, `armed`, `stopped`, `pong`, `error`, `cal_start`, `cal_step`, `cal_done`.
- **Angoli Tinker**: J1/3/5 ∈ [-135,135], J2/4/6 ∈ [-90,90] (conversione hardware dentro il .ino).
- **Timeout server**: nessun `state` per 4s → warning; >6s → chiusura porta e riconnessione (single-flight, max 1 connect alla volta, generation token per evitare Access denied).

### 1.4 API HTTP principali (porta 8765)

| Endpoint | Funzione |
|---|---|
| `GET /api/esp32/status` | Stato connessione, `q[]`, `armed`, `lastSeen` |
| `GET /api/esp32/logs` | Ultime righe di log seriale |
| `POST /api/esp32/send` | Comando grezzo |
| `POST /api/esp32/reconnect` | Riconnessione manuale (attesa 900 ms rilascio handle Windows) |
| `POST /api/tinker/seg\|arm\|cal` | Comandi strutturati con validazione |
| `GET /api/serial/ports` | Elenco porte COM |
| `GET /api/kinect/status` | Proxy stato bridge Kinect :8766 |
| `POST /api/save_*` | Salvataggio config (poses, anim, physics, speeds, notes, stl) |

### 1.5 Fix applicati il 22.09.2026 (stato di questo backup)

1. **`server.js` — riconnessione seriale robusta**
   - `connectInFlight` (single-flight): max 1 apertura porta alla volta → niente più `Access denied` da tentativi concorrenti
   - `portGeneration`: gli handler `close/error/data` della porta vecchia vengono staccati e invalidati → non azzerano più lo stato della nuova connessione
   - Attesa 400 ms dopo `close` prima di riaprire (rilascio handle Windows)
   - Backoff 2,5 s su `Access denied`
   - Timeout CAL alzato da 90 s a **150 s** (la calibrazione reale dura ~110 s)
2. **`desktop-app/src-tauri/src/main.rs`** — corretto typo path live: `Marc-Codici-6-DOF-main` → `Marco-Codici-6-DOF-main` (prima Tauri lanciava la copia packaged vecchia)
3. **Copia packaged** `desktop-app/.../project/server.js` risincronizzata con la versione live

---

## 2. `firmware/` — Firmware ESP32 CORRETTO

| File | Note |
|---|---|
| **`Robot6DOF_ESP32_FINALE_CORRETTO/Robot6DOF_ESP32_FINALE_CORRETTO.ino`** | **USARE SOLO QUESTO.** ~609 righe. Protocollo JSON `state`. |
| **`README_FIRMWARE_CORRETTO.txt`** | Istruzioni + **storico flash**. |

### Specifiche hardware (dal README firmware)

- **Pin**: J1→23, J2→19 (+14 inverso), J3→25, J4→26, J5→18, J6→13
- **PWM**: dispari 500–2450 µs (270°), pari 500–1800 µs (180°)
- **UART**: Serial USB (COM5) + Serial2 GPIO16/17 @115200
- **Board Arduino**: ESP32 Dev Module
- **Libreria**: ESP32Servo

### Storico flash (aggiornare a ogni nuovo flash!)

```
22.09.2026 - Flashato da:
  C:\Users\Marco Trulli\Desktop\Robot6DOF_ESP32_FLASH\Robot6DOF_ESP32_FINALE_CORRETTO.ino
  Fix inclusi: filtro "cal" nei due rami di loop(); handleCal non-bloccante
  con delayWithState() che continua a inviare state e ascolta STOP.
```

### Fix nel .ino rispetto alla sorgente storica `robot6dof_final_tinker.ino`

1. `loop()` (entrambi i rami Serial e Serial2): il filtro accetta anche `cal` (prima un `CAL ...` cadeva in "comando non riconosciuto" e il PC andava in timeout)
2. `handleCal`: le `delay(5000)` sono sostituite da `delayWithState(ms)` → invia `state` ogni 100 ms durante le pause e intercetta `STOP`
3. `pending` impostato durante CAL (timeout firmware 130 s)

---

## 3. `foto braccio/` — Foto del braccio reale

- **26 immagini JPEG** esportate da WhatsApp, data **22 settembre 2026** (orari 18:15–18:58)
- Raffigurano il **braccio robotico fisico** montato: base nera conica, segmenti bianchi stampati in 3D, servi neri/rossi, cavi, ESP32 con LED, alimentatore a griglia, alimentato e attivo
- **Uso previsto**: dataset iniziale per la prossima feature (riconoscimento giunti tramite Kinect/ML — piano non ancora implementato)
- **Limite**: 26 foto non bastano per trainare un modello ML da servirsi; servirà auto-capture dalla Kinect

---

## 4. `desktop-app/` — App desktop Tauri

| Elemento | Ruolo |
|---|---|
| `src-tauri/src/main.rs` | Launcher: cerca la cartella live (path corretto), fa `spawn node server.js`, apre `http://localhost:8765/ik_simulator_v30.html` |
| `src-tauri/tauri.conf.json`, `Cargo.toml` | Config build Tauri |
| `src-tauri/icons/` | Icone app |
| `dist/index.html`, `dist/error.html` | Pagine di boot/errore del webview |
| `package.json` | Config npm del wrapper |
| `src-tauri/target/release/project/` | **Copia packaged** di server.js + assets (risincronizzata con live il 22.09.2026) — usata solo se il path live non viene trovato |

**Non inclusa**: `src-tauri/target/` (~1,25 GB) — ricostruire con `cargo build --release` in `src-tauri/` se serve l'exe.

---

## 5. `Robot6DOF_ESP32_FLASH/`

Copia del `.ino` depositata sul Desktop per il **flash manuale via Arduino IDE** del 22.09.2026 (come da richiesta utente). Contenuto identico alla cartella `firmware/Robot6DOF_ESP32_FINALE_CORRETTO/` al momento del backup.

---

## 6. `test_servo/` — Laboratorio storico (origine della logica)

Cartella di sviluppo da cui nasce il firmware. Principi logici ereditati:

| File | Contributo |
|---|---|
| **`robot6dof_final_tinker/robot6dof_final_tinker.ino`** | **Sorgente dichiarata** del firmware corretto (protocollo JSON, SEG/ARM/STOP/PING/CAL) |
| **`test_servo_finale/test_servo_finale.ino`** | Pin, PWM, S-curve `muoviServoFluido`, `VELOCITA_MASTER`, slow-zone J2, comandi `j1 90g` |
| `test_servo_fluido/`, `test_servo_corretto/`, `mappatura_corretta/` | Iterazioni intermedie |
| `test_servo.ino` | Primo test servos |
| `firmware_generator_new.cpp/.h` | Generatore sperimentale |

---

## COME RIPRISTINARE

1. Copiare questo intero backup in una cartella pulita (es. Desktop)
2. `cd Marco-Codici-6-DOF-main && npm install`
3. Assicurarsi che ESP32 sia collegato (COM5) con il firmware di `firmware/` o `Robot6DOF_ESP32_FLASH/`
4. Avviare: `node server.js` oppure l'app desktop Tauri
5. Browser: `http://localhost:8765/ik_simulator_v30.html`
6. Per Kinect: `kinect/KinectBridge.exe` parte da solo con il server (SDK 1.8 installato)

---

## CONTESTO AL 22.09.2026 (storia della sessione)

- Risolti timeout ESP32 e `Access denied` COM5 (vecchio server packaged + heartbeat annidato + typo path Tauri)
- Flashato firmware corretto con fix CAL
- Aggiornato `README_FIRMWARE_CORRETTO.txt` con storico flash e istruzione per agenti futuri
- Pianificata (non implementata) feature **vista Kinect → riconoscimento braccio robotico → giunti finti 3D** (overlay RGB + scheletro 3D), con approccio ML su keypoints a 7 punti

---

*Prossimo agente: per ogni nuovo flash ESP32 aggiornare `firmware/README_FIRMWARE_CORRETTO.txt`; per ogni nuovo backup creare un README_BACKUP analogo a questo con data e elenco completo.*
