#!/bin/bash

# Robot 6 DOF Control Center - Script Installazione Raspberry Pi
# Questo script installa automaticamente tutte le dipendenze necessarie

set -e

echo "🤖 Robot 6 DOF Control Center - Installazione Raspberry Pi"
echo "=========================================================="

# Controlla se è root
if [ "$EUID" -ne 0 ]; then 
    echo "⚠️  Esegui questo script con sudo: sudo ./install_raspberry.sh"
    exit 1
fi

# Aggiorna sistema
echo "📦 Aggiornamento sistema..."
apt update && apt upgrade -y

# Installa Node.js se non presente
if ! command -v node &> /dev/null; then
    echo "📥 Installazione Node.js..."
    curl -fsSL https://deb.nodesource.com/setup_18.x | bash -
    apt install -y nodejs
else
    echo "✅ Node.js già installato: $(node -v)"
fi

# Installa dipendenze seriali
echo "📥 Installazione dipendenze seriali..."
apt install -y build-essential python3-dev

# Installa dipendenze npm
echo "📦 Installazione dipendenze npm..."
npm install

# Configura permessi utente per porta seriale
echo "🔧 Configurazione permessi seriali..."
USER=$(logname)
usermod -a -G dialout $USER
echo "⚠️  Riavvia il Raspberry Pi per applicare i permessi seriali"

# Crea script di avvio facile
echo "🚀 Creazione script di avvio..."
cat > /usr/local/bin/robot6dof << 'EOF'
#!/bin/bash
cd /home/pi/robot6dof
node server.js
EOF
chmod +x /usr/local/bin/robot6dof

# Crea servizio systemd per avvio automatico
echo "⚙️  Configurazione servizio systemd..."
cat > /etc/systemd/system/robot6dof.service << EOF
[Unit]
Description=Robot 6 DOF Control Center
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/robot6dof
ExecStart=/usr/bin/node server.js
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable robot6dof.service

echo ""
echo "✅ Installazione completata!"
echo "=========================================================="
echo "📌 Prossimi passi:"
echo "1. Riavvia il Raspberry Pi: sudo reboot"
echo "2. Dopo il riavvio, il servizio partirà automaticamente"
echo "3. Accesso web: http://<IP-RASPBERRY>:8765"
echo "4. Per avvio manuale: robot6dof"
echo "5. Per stato servizio: sudo systemctl status robot6dof"
echo "=========================================================="
echo "📁 Nuova struttura file:"
echo "  - config/     : file JSON configurazione"
echo "  - models/     : file STL 3D"
echo "  - firmware/   : file .ino ESP32"
echo "  - scripts/    : script .bat e .sh"
echo "  - docs/       : documentazione"
echo "  - dist/       : eseguibili e zip"
echo "=========================================================="
