# 🤖 Software completo — Braccio robotico 6 DOF

Repository con **tutto** il software del progetto: simulatore 3D (Digital Twin),
server Node.js, app desktop (Tauri), visione artificiale/ML, firmware ESP32,
modelli STL, dataset, backup e documentazione.

**Repo:** <https://github.com/marcotrulli/software-completo-braccio>

---

## ⚠️ Quale cartella devo usare?

Il repository contiene **tre copie del progetto**: una "originale" e due
pronte all'uso, perché i percorsi assoluti (`C:\Users\...`) di un PC non
funzionano sugli altri computer.

| Se sei su… | Usa questa cartella | Perché |
|---|---|---|
| **PC Windows (qualsiasi)** | `OS/windows/` | Copia con percorsi generici, funziona dopo un semplice `git pull` |
| **Linux — Fedora (il tuo portatile)** | `OS/linux/` | Copia con percorsi generici **+** adattamenti Linux (Python, porte seriali, script `.sh`) |
| PC di sviluppo di Marco (questo PC) | cartella principale (radice del repo) | È il progetto com'è in locale, mantiene i percorsi assoluti di questo PC |

> **Regola semplice:** su un altro computer usa **sempre** `OS/windows` oppure
> `OS/linux`. La cartella principale serve da "master" di riferimento.

Dentro `OS/windows` e `OS/linux` c'è **l'intero progetto**, identico alla
radice (stessi file, stesse cartelle): entra nella cartella
`Marco-Codici-6-DOF-main` e avvia il server da lì.

---

## 📁 Struttura del repository

```
software-completo-braccio/                  ← radice del repo (= questa cartella)
├── README.md                               ← questo file
├── .gitignore                              ← cosa NON viene caricato su GitHub
│
├── Marco-Codici-6-DOF-main/                ← PROGETTO PRINCIPALE (com'è sul PC di sviluppo)
│   ├── ik_simulator_v30.html               ← simulatore 3D / interfaccia
│   ├── index.html                          ← interfaccia web
│   ├── server.js                           ← backend Node.js (porta 8765)
│   ├── Program.cs                          ← backend C# "DigitalTwin6DOF" (solo Windows)
│   ├── config/                             ← pose, animazioni, configurazione STL
│   ├── models/                             ← modelli STL del braccio
│   ├── libs/                               ← librerie JS (Three.js, STLLoader…)
│   ├── vision/                             ← visione artificiale: dataset, ML, modello ONNX
│   ├── firmware/                           ← firmware ESP32 (.ino)
│   ├── scripts/                            ← script di setup/avvio (.bat e .sh)
│   ├── docs/                               ← documentazione
│   └── simulator_versions/                 ← versioni storiche del simulatore
│
├── desktop-app/                            ← app desktop Tauri (Rust + webview)
│   └── src-tauri/                          ← sorgenti Rust e configurazione bundle
├── firmware/                               ← firmware ESP32 "corretto" + README flash
├── foto braccio/                           ← foto di riferimento del braccio
├── BACKUP_20260922_191516/                 ← backup datati (archivio storico)
├── Marco-Codici-6-DOF-main_BACKUP_20260919_172941/
│
└── OS/                                     ← ⭐ LE DUE VERSIONI PORTABILI
    ├── windows/                            ← copia completa, pronta per Windows
    │   ├── README_WINDOWS.md
    │   ├── Marco-Codici-6-DOF-main/        ← stessa identica struttura
    │   ├── desktop-app/
    │   └── …
    └── linux/                              ← copia completa, pronta per Linux
        ├── README_LINUX.md                 ← guida passo-passo per Fedora
        ├── Marco-Codici-6-DOF-main/        ← stessa identica struttura
        ├── desktop-app/
        └── …
```

---

## 🔧 Perché esistono tre copie? (i "collegamenti" resi generici)

Nel progetto originale alcuni file contengono **indirizzi assoluti del PC di
sviluppo**: se fai `git pull` su un altro computer quei percorsi non esistono e
qualcosa si rompe. Nelle cartelle `OS/` quegli stessi punti sono stati resi
**generici** (relativi o automatici).

| File | Progetto principale (invariato) | `OS/windows` e `OS/linux` (reso generico) |
|---|---|---|
| `desktop-app/src-tauri/src/main.rs` | `C:\Users\Marco Trulli\Desktop\...\Marco-Codici-6-DOF-main` | percorso vuoto: la cartella del progetto viene **trovata automaticamente** risalendo dalle cartelle dell'eseguibile (oppure con la variabile `ROBOT6DOF_PROJECT_DIR`) |
| `Marco-Codici-6-DOF-main/server.js` | `.venv\Scripts\python.exe` (solo Windows) e porta `COM5` | **Windows:** `Scripts\python.exe` + `COM5` · **Linux:** `.venv/bin/python3` + `/dev/ttyUSB0` (entrambi sovrascrivibili con `ROBOT6DOF_PREFERRED_PORT`) |
| `vision/ml/yolo_dataset/data.yaml` | `path: C:/Users/Marco Trulli/Desktop/...` | `path: .` (percorso relativo al file) |
| `vision/ml/train.py` | scrive un `data.yaml` con percorso assoluto | scrive `path: .` |
| `vision/ml/README.md` | `.venv\Scripts\activate` | **Linux:** `source .venv/bin/activate`, `python3 -m venv` |
| `docs/DEPLOYMENT_RASPBERRY.md` | `cd "C:\Users\Barbara\..."` | `cd "C:\percorso\del\tuo\progetto"` (Windows) · `cd ~/percorso/del/tuo/progetto` (Linux) |
| `firmware/README_FIRMWARE_CORRETTO.txt` | `C:\Users\Marco Trulli\Desktop\...` | `<cartella locale>/...` |
| URL GitHub nei README/script | vecchi repo (`Marco-Codici-6-DOF`, `SoftwareRasp`) | `https://github.com/marcotrulli/software-completo-braccio` |
| `desktop-app/src-tauri/tauri.conf.json` | bundle Windows `nsis` | **Linux:** bundle `deb`, `rpm`, `appimage` |
| `Marco-Codici-6-DOF-main/Program.cs` | apre `explorer.exe` | **Linux:** chiamata protetta + nota "launcher solo Windows" |
| Script di avvio | solo `.bat` | **Linux:** aggiunto `scripts/AVVIA_SIMULATORE.sh` |

Le cartelle `BACKUP_*` sono **archivio storico**: non vengono toccate e
mantengono i percorsi originali (servono solo come backup).

---

## 🚀 Avvio rapido

### Windows (`OS/windows`)

```bat
cd OS\windows\Marco-Codici-6-DOF-main
scripts\setup.bat          :: una volta sola (installa le dipendenze Node)
scripts\AVVIA_SIMULATORE.bat
```

Poi apri <http://localhost:8765>.  
In alternativa: `scripts\start.bat`.

### Linux / Fedora (`OS/linux`)

```bash
sudo dnf install nodejs npm          # una volta sola
cd OS/linux/Marco-Codici-6-DOF-main
npm install                          # una volta sola
./scripts/AVVIA_SIMULATORE.sh        # avvia server + apre il browser
```

Poi apri <http://localhost:8765>.  
Guide complete: [`OS/windows/README_WINDOWS.md`](OS/windows/README_WINDOWS.md) e
[`OS/linux/README_LINUX.md`](OS/linux/README_LINUX.md).

---

## 🧩 Componenti opzionali

### 1. Visione artificiale / ML (`vision/ml`)

```bash
# Linux
cd vision/ml && python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Windows (cmd)
cd vision\ml
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

Il server ri-allena da solo il modello quando arrivano nuovi campioni
(`vision/ml/auto_train.log`); il modello finale è
`vision/model/model.keypoints.onnx`.

### 2. App desktop (`desktop-app`, Tauri)

```bash
cd desktop-app
npm install
npm run dev        # sviluppo
npm run build      # bundle installabile
```

- **Windows:** installer NSIS (`.exe`)
- **Linux:** pacchetti `deb`/`rpm`/`appimage`  
  prerequisiti Fedora: `sudo dnf install webkit2gtk4.1 gtk3 rust cargo`

### 3. Firmware ESP32 (`firmware/`)

Si flasha con Arduino IDE → *ESP32 Dev Module* → porta seriale
(`COM5` su Windows, `/dev/ttyUSB0` su Linux). Dettagli in
`firmware/README_FIRMWARE_CORRETTO.txt`.

### 4. Solo Windows

- **Kinect** (`kinect/KinectBridge.exe`) richiede *Kinect for Windows SDK 1.8*:
  su Linux il server la rileva assente e parte in modalità senzasensori.
- **`Program.cs` / `DigitalTwin6DOF.exe`**: launcher C# con tray icon (WinForms),
  eseguibile solo su Windows. Su Linux usa `node server.js`.

---

## 📦 Cosa NON è caricato su GitHub (e come si ricrea)

Questi elementi sono artefatti generati o ricreati con un comando; caricarli
renderebbe il repo enorme (il solo `.venv` pesa 5 GB) e non servono a nessuno:

| Escluso | Peso | Come ricreare |
|---|---|---|
| `node_modules/` | ~15 MB | `npm install` |
| `vision/ml/.venv/` | ~5 GB | `python3 -m venv .venv` + `pip install -r requirements.txt` |
| `desktop-app/src-tauri/target/` | ~1,2 GB | `cargo build` / `npm run build` |
| `dist/`, `build/`, `bin/`, `obj/` | — | build del progetto |
| `*.log`, `*.cache` | — | rigenerati in esecuzione |

Tutto il resto (codice, modelli STL, foto, dataset, pesi `.pt`/`.onnx`,
backup, documentazione) **è** nel repository. Dimensione indicativa: ~460 MB
(sono inclusi dataset fotografici e output di training).

---

## 🔌 Variabili d'ambiente utili

| Variabile | Default | Scopo |
|---|---|---|
| `ROBOT6DOF_PROJECT_DIR` | rilevato automaticamente | forza il percorso della cartella del progetto (serve solo se il rilevamento automatico non basta) |
| `ROBOT6DOF_PREFERRED_PORT` | `COM5` (Windows) / `/dev/ttyUSB0` (Linux) | porta seriale dell'ESP32 da usare per prima |
| `VISION_ML_AUTO` | attivo | `0` disattiva il ri-allenamento automatico del modello |
| `VISION_ML_AUTO_MIN` | `20` | campioni nuovi necessari per ri-allenare |

---

## 📖 Documentazione interna

- `Marco-Codici-6-DOF-main/README.md` — guida del progetto
- `Marco-Codici-6-DOF-main/docs/QUICKSTART.md` — guida rapida
- `Marco-Codici-6-DOF-main/docs/DEPLOYMENT_RASPBERRY.md` — deploy su Raspberry Pi
- `Marco-Codici-6-DOF-main/docs/README.md` — documentazione completa
- `OS/linux/README_LINUX.md` — installazione su Fedora

## 📄 Licenza

MIT License
