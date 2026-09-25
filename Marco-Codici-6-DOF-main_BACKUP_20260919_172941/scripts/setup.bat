@echo off
title Robot 6 DOF - Setup Automatico
cd /d "%~dp0.."

echo ============================================
echo   Robot 6 DOF - Setup Automatico
echo ============================================
echo.

REM Controlla Node.js
node --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Node.js non installato
    echo.
    echo 📥 Installa Node.js da: https://nodejs.org/
    echo 🌐 Apre il sito di download...
    start https://nodejs.org/
    echo.
    echo ⚠️  Dopo l'installazione, riavvia questo script
    pause
    exit /b 0
)

echo ✅ Node.js trovato
node --version
echo.

REM Installa dipendenze
if exist "node_modules\" (
    echo ✅ Dipendenze già installate
) else (
    echo 📦 Installazione dipendenze...
    call npm install
    if errorlevel 1 (
        echo ❌ Errore installazione
        pause
        exit /b 1
    )
    echo ✅ Dipendenze installate
)

echo.
echo ============================================
echo ✅ Setup completato!
echo.
echo 🚀 Per avviare:
echo    npm start
echo.
pause
