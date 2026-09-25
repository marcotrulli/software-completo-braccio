#!/bin/bash

# Robot 6 DOF Control Center - Script avvio manuale Raspberry Pi
# Uso: ./start_raspberry.sh

cd "$(dirname "$0")/.."

echo "🚀 Avvio Robot 6 DOF Control Center..."
echo "📡 Server su http://localhost:8765"
echo "🌐 Accessibile da rete: http://$(hostname -I | awk '{print $1}'):8765"
echo ""

# Verifica Node.js
if ! command -v node &> /dev/null; then
    echo "❌ Node.js non installato. Esegui: sudo ./scripts/install_raspberry.sh"
    exit 1
fi

# Verifica dipendenze
if [ ! -d "node_modules" ]; then
    echo "📦 Installazione dipendenze..."
    npm install
fi

# Avvia server
node server.js
