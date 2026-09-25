# 🐧 Versione LINUX — progetto pronto per Fedora

Questa cartella è una **copia completa e identica** del progetto principale,
con i percorsi resi **generici** e i componenti **adattati a Linux**: puoi
clonare il repository sul portatile Fedora e farlo funzionare senza toccare i
sorgenti.

## 1. Prerequisiti (una volta sola)

```bash
# Fedora
sudo dnf install git nodejs npm python3 python3-pip

# opzionale: app desktop Tauri
sudo dnf install webkit2gtk4.1 gtk3 rust cargo

# opzionale: permessi sulla porta seriale dell'ESP32
sudo usermod -aG dialout $USER    # poi disconnetti e riconnetti
```

Su Debian/Ubuntu: `sudo apt install git nodejs npm python3`.

## 2. Clone + avvio

```bash
git clone https://github.com/marcotrulli/software-completo-braccio.git
cd software-completo-braccio/OS/linux/Marco-Codici-6-DOF-main

npm install                       # una volta sola
./scripts/AVVIA_SIMULATORE.sh     # avvia il server e apre il browser
```

Poi apri <http://localhost:8765>.

Se lo script non è eseguibile dopo il clone:

```bash
chmod +x scripts/*.sh
```

Alternative: `bash scripts/setup.sh` (setup) e `node server.js` (solo server).

## 3. Seriale ESP32 su Linux

- La porta si chiama `/dev/ttyUSB0` (o `/dev/ttyACM0`), non `COM5`.
- Il server la riconosce da sola; per forzarne una:
  `ROBOT6DOF_PREFERRED_PORT=/dev/ttyUSB0 node server.js`
- Se negato l'accesso: `sudo usermod -aG dialout $USER` (poi rilogga).
- Per i permessi in alternativa: `sudo chmod a+rw /dev/ttyUSB0`

## 4. Visione artificiale / ML (opzionale)

```bash
cd vision/ml
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Il server rileva automaticamente `.venv/bin/python3` (su Windows sarebbe
`.venv\Scripts\python.exe`). Modello prodotto: `vision/model/model.keypoints.onnx`.

## 5. App desktop Tauri (opzionale)

```bash
cd desktop-app
npm install
npm run dev        # sviluppo
npm run build      # produce deb/rpm/appimage
```

Il bundle è impostato su `deb`, `rpm`, `appimage` (su Windows sarebbe NSIS).

## 6. Firmware ESP32

Arduino IDE (o `arduino-cli`) → scheda *ESP32 Dev Module* → porta
`/dev/ttyUSB0`. Istruzioni in `firmware/README_FIRMWARE_CORRETTO.txt`.

---

## ⚠️ Cosa NON funziona su Linux

| Componente | Motivo | Alternativa |
|---|---|---|
| `kinect/KinectBridge.exe` | richiede *Kinect for Windows SDK 1.8* | il server parte in modalità "senza Kinect" (simulazione) |
| `Program.cs` / `DigitalTwin6DOF.exe` | launcher C# con tray icon WinForms | `node server.js` oppure `scripts/AVVIA_SIMULATORE.sh` |
| Script `.bat` (`setup.bat`, `start.bat`, `AVVIA_SIMULATORE.bat`, `PREPARA_GITHUB.bat`, `SINCRONIZZA_DA_GITHUB.bat`) | batch file Windows | gli equivalenti `.sh` (`setup.sh`, `AVVIA_SIMULATORE.sh`, `start_raspberry.sh`) |
| Cartelle `BACKUP_*` | archivio storico con i percorsi originali | servono solo come backup, non vanno usati |

Tutto il resto (server, simulatore 3D, pose/animazioni, modelli STL,
dataset, ML, documentazione) funziona.

## 🔧 Risoluzione problemi

| Problema | Soluzione |
|---|---|
| `command not found: node` | `sudo dnf install nodejs` |
| `Cannot find module 'express'` | esegui `npm install` nella cartella del progetto |
| Errore permessi su `/dev/ttyUSB0` | `sudo usermod -aG dialout $USER` + rilogga |
| La venv ML non viene trovata | ricreala: `python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt` |
| Cartella del progetto non trovata (app desktop) | esporta `ROBOT6DOF_PROJECT_DIR=/percorso/.../OS/linux/Marco-Codici-6-DOF-main` |
| I file `.sh` non si eseguono | `chmod +x scripts/*.sh` |
| Il browser non si apre da solo | apri manualmente <http://localhost:8765> |
