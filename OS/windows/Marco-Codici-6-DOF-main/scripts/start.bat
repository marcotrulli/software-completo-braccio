@echo off
title Robot 6 DOF
cd /d "%~dp0.."

echo 🚀 Avvio Robot 6 DOF...
echo.

REM Controlla setup
if not exist "node_modules\" (
    echo ⚠️  Prima esecuzione - esegui setup
    call scripts\setup.bat
)

REM Avvia server
node server.js

echo.
echo Il server si e' arrestato.
pause
