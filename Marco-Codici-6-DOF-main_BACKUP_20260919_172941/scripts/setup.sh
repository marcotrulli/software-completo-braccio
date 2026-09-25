#!/bin/bash

echo "============================================"
echo "  Robot 6 DOF - Setup Automatico"
echo "============================================"
echo ""

# Controlla Node.js
if ! command -v node &> /dev/null; then
    echo "❌ Node.js non installato"
    echo "📥 Installa Node.js: curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && apt install -y nodejs"
    exit 1
fi

echo "✅ Node.js trovato"
node --version
echo ""

# Installa dipendenze
if [ -d "node_modules" ]; then
    echo "✅ Dipendenze già installate"
else
    echo "📦 Installazione dipendenze..."
    npm install
    if [ $? -ne 0 ]; then
        echo "❌ Errore installazione"
        exit 1
    fi
    echo "✅ Dipendenze installate"
fi

echo ""
echo "============================================"
echo "✅ Setup completato!"
echo ""
echo "🚀 Per avviare: npm start"
echo ""
