# 🤖 Robot 6 DOF Control Center

Sistema completo per simulazione e controllo hardware di braccio robotico 6 DOF con interfaccia web, fisica avanzata e comunicazione seriale ESP32.

## 🌟 Caratteristiche

- **Digital Twin 3D**: Simulazione fotorealistica con Three.js
- **Cinematica Inversa 6 DOF**: Risolutore ibrido analitico + DLS
- **Fisica Avanzata**: Calcolo coppie gravitazionali e analisi FEA materiali
- **Doppia Modalità**: 
  - **PC**: Solo simulazione (server C# `DigitalTwin6DOF.exe`)
  - **Raspberry Pi**: Controllo hardware reale via seriale ESP32
- **Controllo Manuale**: Invio comandi all'ESP32 solo dopo conferma utente

## 📁 Struttura Progetto

```
Marco Codici 6 DOF/
├── index.html                          # Applicazione web principale
├── ik_simulator_v30.html               # Versione corrente (alias)
├── Program.cs                          # Server C# per PC (Windows)
├── server.js                           # Server Node.js per Raspberry Pi
├── package.json                        # Dipendenze Node.js
├── libs/                               # Librerie Three.js offline
├── config/                             # File JSON configurazione
│   ├── robot6dof_stl_config.json
│   ├── robot6dof_saved_poses.json
│   └── robot6dof_animations.json
├── models/                             # File STL 3D
│   ├── base_finita.stl
│   ├── braccio_1.stl
│   └── ...
├── firmware/                           # Firmware ESP32
│   └── Robot6DOF_ESP32_Firmware.ino
├── scripts/                            # Script di installazione/avvio/test
│   ├── install_raspberry.sh
│   ├── start_raspberry.sh
│   ├── AVVIA_SIMULATORE.bat
│   ├── TEST_PC_SERVER.bat
│   ├── TEST_NODE_SERVER.bat
│   └── test_esp32_simulation.js
├── docs/                               # Documentazione
│   ├── README.md
│   ├── QUICKSTART.md
│   ├── DEPLOYMENT_RASPBERRY.md
│   ├── CONTRIBUTING.md
│   └── ROBOT_6DOF_SYSTEM_PROMPT.md
├── dist/                               # Eseguibili e build
│   ├── DigitalTwin6DOF.exe
│   └── DigitalTwin6DOF_Portable.zip
└── .gitignore                          # File da ignorare in Git
```

## 🚀 Avvio Rapido

### Su PC (Windows)

1. **Verifica struttura**:
   ```bash
   scripts\TEST_PC_SERVER.bat
   ```

2. **Avvia il simulatore**:
   ```bash
   scripts\AVVIA_SIMULATORE.bat
   ```
   Oppure esegui direttamente `dist\DigitalTwin6DOF.exe`

3. Il browser si aprirà automaticamente su `http://localhost:8765`

4. Usa l'interfaccia in modalità **Solo Simulazione**

### Su Raspberry Pi (Controllo Hardware)

1. **Clona il repository**:
   ```bash
   cd ~
   git clone <tuo-repo-url> robot6dof
   cd robot6dof
   ```

2. **Verifica struttura**:
   ```bash
   npm run install-raspberry
   # oppure manualmente:
   sudo ./scripts/install_raspberry.sh
   ```

3. **Riavvia il Raspberry Pi**:
   ```bash
   sudo reboot
   ```

4. **Connetti ESP32** via USB al Raspberry Pi

5. **Accedi all'interfaccia web**:
   ```
   http://<IP-RASPBERRY>:8765
   ```

6. **Attiva modalità hardware**:
   - Vai nel tab "🤖 ESP32 Control"
   - Seleziona "Controllo Hardware (Raspberry Pi)"
   - Verifica che ESP32 sia connesso (stato verde)
   - Usa i pulsanti per inviare comandi manualmente

## 🔧 Comandi ESP32 Supportati

Il firmware ESP32 accetta i seguenti comandi via seriale (115200 baud):

### Comandi Manuali
- `IK <X> <Y> <Z> [0|1]` - Cinematica inversa alle coordinate (1 = orizzontale)
  ```
  IK 150.0 200.0 120.0 1
  ```
- `MOVE <J1> <J2> <J3> <J4> <J5> <J6>` - Movimento diretto giunti in gradi
  ```
  MOVE 0 15 0 -85 0 70
  ```
- `REST` - Ritorna alla posa di riposo `[0, 47, 0, -69, 7, -68]`
- `STATUS` o `POS` - Mostra stato attuale (angoli, coordinate, stress)
- `ATTACH` / `DETACH` - Attiva/disattiva segnali PWM servomotori
- `HELP` - Mostra menu comandi

### Invio dall'Interfaccia Web

Nella sezione "🤖 ESP32 Control" puoi:

1. **Comando Manuale**: Scrivi direttamente il comando nell'input
2. **Invia Posa Attuale**: Invia gli angoli attuali del simulatore
3. **Invia REST**: Invia comando di riposo all'ESP32
4. **Log Seriale**: Visualizza cronologia comandi inviati

## 🌐 Architettura

```
┌─────────────────────────────────────────────────────────┐
│  RASPBERRY PI - CONTROL CENTER                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │  App Web (index.html)                           │  │
│  │  - Auto-detect piattaforma                      │  │
│  │  - Modalità: Simulazione | Controllo Hardware    │  │
│  └────────────────────┬─────────────────────────────┘  │
│                       │                                  │
│  ┌────────────────────┴─────────────────────────────┐  │
│  │  Server Node.js (server.js)                     │  │
│  │  - API /api/esp32/send                          │  │
│  │  - Gestione porta seriale                       │  │
│  └────────────────────┬─────────────────────────────┘  │
└─────────────────────┼──────────────────────────────────┘
                      │ USB/Serial
                      ▼
          ┌───────────────────────────┐
          │  ESP32 (Arduino Firmware) │
          │  - 6 Servomotori          │
          │  - CLI seriale 115200     │
          └───────────────────────────┘
```

## 🔍 Diagnostica

### Test Server PC
```bash
# Verifica struttura e file
scripts\TEST_PC_SERVER.bat

# Avvio manuale
dist\DigitalTwin6DOF.exe
```

### Test Server Node.js
```bash
# Verifica struttura e dipendenze
scripts\TEST_NODE_SERVER.bat

# Test comunicazione simulata
npm run test-esp32
# oppure
node scripts/test_esp32_simulation.js
```

### Verifica connessione ESP32
```bash
# Lista porte seriali disponibili
ls /dev/ttyUSB* /dev/ttyACM*

# Monitor seriale ESP32 (per debug)
screen /dev/ttyUSB0 115200
```

### Stato servizio Raspberry Pi
```bash
# Controlla se il servizio è attivo
sudo systemctl status robot6dof

# Riavvia il servizio
sudo systemctl restart robot6dof

# Log del servizio
sudo journalctl -u robot6dof -f
```

### Risoluzione problemi comuni

**ESP32 non rilevato:**
- Verifica cavo USB
- Controlla permessi: `sudo usermod -a -G dialout $USER`
- Riavvia dopo aggiunta utente al gruppo dialout

**Servizio non parte:**
- Controlla log: `sudo journalctl -u robot6dof -n 50`
- Verifica Node.js installato: `node -v`
- Controlla dipendenze: `npm install`

**Comandi non arrivano all'ESP32:**
- Verifica baudrate (deve essere 115200)
- Controlla log seriale nell'interfaccia web
- Prova invio manuale via `screen` per debug

## 📝 Sync tra PC e Raspberry Pi

Il file `index.html` è **identico** su entrambe le piattaforme:

- **Su PC**: Usa server C# (`Program.cs`) - modalità solo simulazione
- **Su Raspberry Pi**: Usa server Node.js (`server.js`) - modalità controllo hardware

L'interfaccia auto-detecta la piattaforma e mostra/nasconde i controlli ESP32 appropriati.

## 🔄 Aggiornamento Software

Per aggiornare il software sul Raspberry Pi:

```bash
cd ~/robot6dof
git pull
sudo systemctl restart robot6dof
```

## 🤝 Contributi

Quando lavori al codice:

1. **Non violare limiti fisici giunti** (270° dispari, 180° pari)
2. **Mantieni sync** tra `index.html`, `ik_simulator_v30.html` e firmware
3. **Preserva calcolo vettoriale** delle forze (non semplificare)
4. **Testa su entrambe piattaforme** prima di commit

## 📄 Licenza

MIT License - Marco Codici 6 DOF

## 🆘 Supporto

Per problemi o domande:
- Controlla il log seriale nell'interfaccia web
- Verifica i log del servizio systemd
- Controlla la connessione fisica ESP32-Raspberry Pi
