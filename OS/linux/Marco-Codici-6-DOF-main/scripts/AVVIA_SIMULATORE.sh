#!/usr/bin/env bash
# Avvio rapido del simulatore 3D su Linux (Fedora, Debian, Raspberry Pi OS).
# Equivalente Linux di scripts/AVVIA_SIMULATORE.bat
set -e
cd "$(dirname "$0")/.."

if ! command -v node >/dev/null 2>&1; then
    echo "[ERRORE] Node.js non installato."
    echo "  Fedora : sudo dnf install nodejs npm"
    echo "  Debian : sudo apt install nodejs npm"
    echo "  Altro  : https://nodejs.org/ (versione LTS)"
    exit 1
fi

echo "Node.js: $(node --version)"

if [ ! -d node_modules ]; then
    echo "Installazione dipendenze in corso, attendere..."
    npm install
fi

echo "Avvio del server e del simulatore 3D su http://localhost:8765 ..."
( sleep 2; xdg-open http://localhost:8765 ) >/dev/null 2>&1 || true
exec node server.js
