# 🚀 GUIDA DEPLOYMENT RASPBERRY PI

## 📋 Prerequisiti

- Raspberry Pi (3/4/5) con Raspberry Pi OS
- Accesso internet per il download delle dipendenze
- ESP32 collegato via USB
- Account GitHub per il clone del repository

## 🔄 PROCEDURA COMPLETA

### 1. Preparazione Repository GitHub

Sul tuo PC (dove sei ora):

```bash
# Inizializza Git se non lo hai già fatto
cd "C:\Users\Barbara\OneDrive\davos\Desktop\Marco Codici 6 DOF"
git init
git add .
git commit -m "Initial commit - Robot 6 DOF Control Center"

# Crea repository su GitHub.com (manuale)
# Poi collega il tuo repository locale:
git remote add origin https://github.com/TUO_USERNAME/robot6dof.git
git branch -M main
git push -u origin main
```

### 2. Setup Raspberry Pi

```bash
# SSH nel Raspberry Pi
ssh pi@<IP-RASPBERRY>

# Aggiorna sistema
sudo apt update && sudo apt upgrade -y

# Clona il repository
cd ~
git clone https://github.com/TUO_USERNAME/robot6dof.git
cd robot6dof

# Esegui installazione automatica
sudo chmod +x scripts/install_raspberry.sh
sudo ./scripts/install_raspberry.sh

# Riavvia per applicare permessi seriali
sudo reboot
```

### 3. Prima Connessione ESP32

Dopo il riavvio:

```bash
# Connetti ESP32 via USB al Raspberry Pi

# Verifica che venga rilevato
ls /dev/ttyUSB* /dev/ttyACM*

# Dovresti vedere qualcosa come /dev/ttyUSB0
```

### 4. Avvio Control Center

```bash
# Il servizio dovrebbe partire automaticamente
# Controlla stato
sudo systemctl status robot6dof

# Se non parte, avvia manualmente
cd ~/robot6dof
./scripts/start_raspberry.sh
```

### 5. Accesso Interfaccia Web

Dal browser (PC o mobile):

```
http://<IP-RASPBERRY>:8765
```

Per trovare l'IP del Raspberry Pi:
```bash
hostname -I
```

### 6. Configurazione Controllo Hardware

1. Apri l'interfaccia web
2. Vai nel tab **"🤖 ESP32 Control"**
3. Seleziona **"Controllo Hardware (Raspberry Pi)"**
4. Verifica che lo stato sia **"✅ ESP32 Connesso"**
5. Usa i pulsanti per inviare comandi manualmente

## 🧪 TEST COMUNICAZIONE

### Test 1: Verifica Connessione ESP32

Nell'interfaccia web:
- Vedi lo stato verde "ESP32 Connesso"?
- Il log mostra messaggi di connessione?

### Test 2: Invio Comando REST

1. Clicca **"🏠 INVIA REST A ESP32"**
2. Guarda il log seriale: dovrebbe mostrare `📤 Inviato: REST`
3. L'ESP32 dovrebbe muovere il braccio nella posa di riposo

### Test 3: Invio Posa Manuale

1. Muovi il braccio nel simulatore (IK o FK)
2. Clicca **"🎯 INVIA POSA ATTUALE A ESP32"**
3. Verifica che l'ESP32 segua la stessa posa

### Test 4: Comando Manuale

1. Scrivi nell'input: `IK 150 200 120 1`
2. Clicca **"📤 INVIA COMANDO A ESP32"**
3. L'ESP32 dovrebbe muovere la pinza alle coordinate specificate

## 🔧 TROUBLESHOOTING

### ESP32 non rilevato

```bash
# Controlla porte seriali
ls -la /dev/ttyUSB* /dev/ttyACM*

# Se non ci sono porte:
# - Verifica cavo USB
# - Prova diversa porta USB
# - Controlla che ESP32 sia alimentato

# Se ci sono porte ma non funzionano:
sudo usermod -a -G dialout $USER
sudo reboot
```

### Servizio non parte

```bash
# Controlla log
sudo journalctl -u robot6dof -n 50

# Riavvia servizio
sudo systemctl restart robot6dof

# Avvio manuale per debug
cd ~/robot6dof
node server.js
```

### Comandi non arrivano

```bash
# Monitor seriale diretto per debug
sudo apt install screen
screen /dev/ttyUSB0 115200

# Dovresti vedere i comandi arrivare in tempo reale
# Esci con Ctrl+A, K
```

### Interfaccia web non accessibile

```bash
# Verifica che il server sia in ascolto
sudo netstat -tlnp | grep 8765

# Controlla firewall (se attivo)
sudo ufw allow 8765

# Verifica servizio
sudo systemctl status robot6dof
```

## 📝 AGGIORNAMENTI

Per aggiornare il software:

```bash
cd ~/robot6dof
git pull
sudo systemctl restart robot6dof
```

## 🔄 SYNC CON PC

Il software è **identico** su PC e Raspberry Pi:

- **Stesso file `index.html`**
- **Stessa interfaccia 3D**
- **Stesso calcolo cinematica/fisica**
- **Differenza solo nel backend**:
  - PC: `Program.cs` (C#) - solo simulazione
  - Raspberry Pi: `server.js` (Node.js) - controllo hardware

Per sincronizzare modifiche dal PC al Raspberry Pi:

```bash
# Sul PC (dopo modifiche)
git add .
git commit -m "Descrizione modifiche"
git push

# Sul Raspberry Pi
cd ~/robot6dof
git pull
sudo systemctl restart robot6dof
```

## 🎯 PROSSIMI PASSI

1. ✅ Completa il setup GitHub
2. ✅ Deploy su Raspberry Pi
3. ✅ Test comunicazione ESP32
4. ✅ Crea traiettorie e animazioni
5. ✅ Sincronizza tra PC e Raspberry Pi

Buon divertimento con il tuo Robot 6 DOF! 🤖
