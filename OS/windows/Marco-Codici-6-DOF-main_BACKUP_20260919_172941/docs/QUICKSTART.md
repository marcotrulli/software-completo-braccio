# 🚀 Quick Start Guide

Guida rapida per iniziare con Robot 6 DOF Control Center.

## ⚡ Setup Rapido (5 minuti)

### Opzione 1: PC Windows (Solo Simulazione)

1. **Scarica o clona il repository**
2. **Avvia il simulatore**:
   ```bash
   scripts\AVVIA_SIMULATORE.bat
   ```
3. **Apri il browser** su `http://localhost:8765`
4. **Pronto!** Usa l'interfaccia in modalità simulazione

### Opzione 2: Raspberry Pi (Controllo Hardware)

1. **Clona il repository**:
   ```bash
   git clone <tuo-repo-url> robot6dof
   cd robot6dof
   ```

2. **Esegui installazione automatica**:
   ```bash
   sudo ./scripts/install_raspberry.sh
   sudo reboot
   ```

3. **Connetti ESP32** via USB

4. **Accedi all'interfaccia**:
   ```
   http://<IP-RASPBERRY>:8765
   ```

5. **Attiva controllo hardware** nel tab "🤖 ESP32 Control"

## 🧪 Testing

### Test Server PC
```bash
scripts\TEST_PC_SERVER.bat
```

### Test Server Node.js
```bash
scripts\TEST_NODE_SERVER.bat
npm run test-esp32
```

## 📁 Struttura Essenziale

```
robot6dof/
├── index.html              # Interfaccia web
├── server.js               # Backend Raspberry Pi
├── Program.cs              # Backend PC
├── config/                 # Configurazioni JSON
├── models/                 # Modelli 3D STL
├── firmware/               # Firmware ESP32
└── scripts/                # Script utili
```

## 🔑 Funzionalità Principali

- **Digital Twin 3D**: Simulazione fotorealistica
- **Cinematica 6 DOF**: Risolutore IK/FK avanzato
- **Fisica Reale**: Calcolo coppie e FEA materiali
- **Controllo Hardware**: Comunicazione ESP32 via seriale
- **Animazioni**: Editor traiettorie e pose

## 📖 Documentazione Completa

- `docs/README.md` - Documentazione completa
- `docs/DEPLOYMENT_RASPBERRY.md` - Guida deployment Raspberry Pi
- `docs/CONTRIBUTING.md` - Linee guida contributi
- `docs/ROBOT_6DOF_SYSTEM_PROMPT.md` - Specifiche tecniche

## 🆘 Problemi Comuni

**Server non parte?**
- PC: Verifica che `dist/DigitalTwin6DOF.exe` esista
- Raspberry: Controlla `sudo systemctl status robot6dof`

**ESP32 non connesso?**
- Verifica cavo USB
- Controlla permessi: `sudo usermod -a -G dialout $USER`
- Riavvia dopo permessi

**Interfaccia non carica?**
- Verifica server su porta 8765
- Controlla firewall
- Prova browser diverso

## 🌐 Accesso Web

- **Locale**: `http://localhost:8765`
- **Rete**: `http://<IP-SERVER>:8765`
- **Raspberry Pi**: `http://<IP-RASPBERRY>:8765`

## 🎯 Prossimi Passi

1. ✅ Setup completato
2. 🎮 Esplora l'interfaccia 3D
3. 🧪 Testa cinematica IK/FK
4. ⚡ Verifica calcoli fisici
5. 🤖 Configura ESP32 (Raspberry Pi)
6. 🎬 Crea animazioni
7. 📊 Esporta dati per robot reale

Buon divertimento con il tuo Robot 6 DOF! 🤖
