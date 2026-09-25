# 🪟 Versione WINDOWS — progetto pronto all'uso

Questa cartella è una **copia completa e identica** del progetto principale,
con i percorsi resi **generici**: puoi clonare il repository su **qualsiasi PC
Windows** e farla funzionare senza modificare nulla.

## Avvio

Prerequisiti: [Node.js LTS](https://nodejs.org/) (porta con sé `npm`).

```bat
cd OS\windows\Marco-Codici-6-DOF-main
scripts\setup.bat
scripts\AVVIA_SIMULATORE.bat
```

Il server parte su <http://localhost:8765> e apre il simulatore 3D.

Alternative:

| Comando | Cosa fa |
|---|---|
| `scripts\start.bat` | avvia solo il server Node |
| `scripts\setup.bat` | installa le dipendenze (`npm install`) |
| `scripts\SINCRONIZZA_DA_GITHUB.bat` | tiene la cartella sincronizzata con GitHub (pull ogni 30 s) |
| `PREPARA_GITHUB.bat` | crea lo zip del progetto da caricare |

## Cosa è diverso dal progetto principale

| File | Modifica |
|---|---|
| `desktop-app\src-tauri\src\main.rs` | rimosso il percorso assoluto `C:\Users\Marco Trulli\...`: la cartella del progetto viene cercata automaticamente (oppure `ROBOT6DOF_PROJECT_DIR`) |
| `Marco-Codici-6-DOF-main\vision\ml\yolo_dataset\data.yaml` | `path: .` invece del percorso assoluto del dataset |
| `Marco-Codici-6-DOF-main\vision\ml\train.py` | riscrive il `data.yaml` con percorso relativo |
| `Marco-Codici-6-DOF-main\docs\DEPLOYMENT_RASPBERRY.md` | esempio di `cd` generico (`C:\percorso\del\tuo\progetto`) |
| `firmware\README_FIRMWARE_CORRETTO.txt` | percorsi di provenienza del firmware resi generici |
| README / script | URL GitHub aggiornati a `marcotrulli/software-completo-braccio` |

Invariati (già corretti per Windows): `.venv\Scripts\python.exe`, porta
`COM5`, script `.bat`, bundle NSIS dell'app desktop.

## Seriale ESP32

Il server usa di default `COM5`; se il tuo PC mette l'ESP32 su un'altra porta
basta impostare `ROBOT6DOF_PREFERRED_PORT=COM3`, oppure **lascia fare al
server**: riconosce da solo la prima porta USB-serial collegata
(CH340/CP210x/FTDI…).

## Opzionale

- **ML / visione artificiale** — `vision\ml\README.md`
- **App desktop** — `cd desktop-app` → `npm install` → `npm run build`
- **Firmware ESP32** — Arduino IDE, cartella `firmware\`
- **Kinect** — serve *Kinect for Windows SDK 1.8* (solo Windows)
