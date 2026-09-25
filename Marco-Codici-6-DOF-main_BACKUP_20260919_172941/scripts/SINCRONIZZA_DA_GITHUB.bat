@echo off
setlocal enabledelayedexpansion

REM Sincronizza automaticamente questa cartella con GitHub.
REM Ogni 30 secondi scarica le modifiche presenti su GitHub.
REM Le modifiche locali non ancora committate vengono messe da parte
REM (autostash) e riapplicate dopo l'aggiornamento.
REM Chiudere la finestra per fermare la sincronizzazione.

cd /d "%~dp0.."

git rev-parse --is-inside-work-tree >nul 2>&1
if errorlevel 1 (
  echo [ERRORE] Questa cartella non e' un repository git.
  echo Clona il progetto con:
  echo     git clone https://github.com/marcotrulli/Marco-Codici-6-DOF.git
  pause
  exit /b 1
)

echo Sincronizzazione attiva su: %CD%
echo Premi CTRL+C oppure chiudi la finestra per fermarla.
echo.

:loop
for /f %%i in ('git rev-parse HEAD') do set "BEFORE=%%i"
git pull --rebase --autostash --quiet
if errorlevel 1 (
  echo [!time!] Sincronizzazione fallita: risolvi i conflitti a mano.
) else (
  for /f %%i in ('git rev-parse HEAD') do set "AFTER=%%i"
  if not "!BEFORE!"=="!AFTER!" echo [!time!] Aggiornato da GitHub.
)
timeout /t 30 /nobreak >nul
goto loop
