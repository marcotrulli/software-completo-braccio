@echo off
title Digital Twin 6 DOF Launcher
cd /d "%~dp0.."

node --version >nul 2>&1
if errorlevel 1 (
    echo [ERRORE] Node.js non e' installato.
    echo Scaricalo da https://nodejs.org/ ^(versione LTS^), poi riavvia questo file.
    start https://nodejs.org/
    pause
    exit /b 1
)

if not exist "node_modules\" (
    echo Installazione dipendenze in corso, attendere...
    call npm install
    if errorlevel 1 (
        echo [ERRORE] Installazione dipendenze fallita.
        pause
        exit /b 1
    )
)

echo Avvio del server e del simulatore 3D su http://localhost:8765 ...
start "" http://localhost:8765
node server.js

echo.
echo Il server si e' arrestato.
pause
