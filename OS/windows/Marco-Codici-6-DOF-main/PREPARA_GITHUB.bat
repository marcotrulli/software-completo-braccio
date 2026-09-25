@echo off
title Preparazione Caricamento GitHub
cd /d "%~dp0.."

set PROJECT_DIR=%cd%

echo ============================================
echo   PREPARAZIONE CARICAMENTO SU GITHUB
echo ============================================
echo Cartella progetto: %PROJECT_DIR%
echo.

echo [1/4] Inizializzazione repo Git...
if not exist .git git init >nul 2>&1
echo      OK

echo [2/4] Creazione commit...
git add -A >nul 2>&1
git commit -m "Robot 6 DOF v30" >nul 2>&1
echo      OK

echo [3/4] Creazione ZIP Robot6DOF_Progetto.zip...

powershell -NoProfile -Command "Set-Location '%PROJECT_DIR%'; $ErrorActionPreference='Stop'; Compress-Archive -Path 'ik_simulator_v30.html','index.html','server.js','Program.cs','package.json','package-lock.json','README.md','ISTRUZIONI_AVVIO_CORRETTO.txt','.gitignore','config\*','scripts\*','firmware\*','models\*','libs\*','docs\*','simulator_versions\*' -DestinationPath 'Robot6DOF_Progetto.zip' -Force -CompressionLevel Optimal"

if %errorlevel% neq 0 (
    echo [ERRORE] PowerShell fallito. Uso 7-Zip se disponibile...
    where 7z >nul 2>&1
    if %errorlevel% equ 0 (
        7z a -tzip "%PROJECT_DIR%\Robot6DOF_Progetto.zip" "%PROJECT_DIR%\ik_simulator_v30.html" "%PROJECT_DIR%\index.html" "%PROJECT_DIR%\server.js" "%PROJECT_DIR%\Program.cs" "%PROJECT_DIR%\package.json" "%PROJECT_DIR%\package-lock.json" "%PROJECT_DIR%\README.md" "%PROJECT_DIR%\ISTRUZIONI_AVVIO_CORRETTO.txt" "%PROJECT_DIR%\.gitignore" "%PROJECT_DIR%\config\*" "%PROJECT_DIR%\scripts\*" "%PROJECT_DIR%\firmware\*" "%PROJECT_DIR%\models\*" "%PROJECT_DIR%\libs\*" "%PROJECT_DIR%\docs\*" "%PROJECT_DIR%\simulator_versions\*" -x!node_modules -x!*.log -x!config\server_*.txt -x!*.tmp -x!.git
        echo [OK] Creato con 7-Zip
        goto :verify
    )
    echo [ERRORE] Nessun metodo funziona. Usa Git diretto:
    echo git init ^& git add -A ^& git commit -m "Robot 6 DOF v30" ^& git remote add origin https://github.com/marcotrulli/software-completo-braccio.git ^& git branch -M main ^& git push -u origin main
    pause
    goto :eof
)

:verify
echo [4/4] Verifica ZIP...
for %%F in (Robot6DOF_Progetto.zip) do (
    echo      File: %%~nxF
    echo      Dimensione: %%~zF bytes
    echo      Percorso: %cd%\%%~nxF
)

echo.
echo ============================================
echo   SUCCESSO! ZIP creato in:
echo   %cd%\Robot6DOF_Progetto.zip
echo ============================================
echo.
echo Carica su GitHub:
echo   https://github.com/marcotrulli/software-completo-braccio
echo   -> Add file -> Upload files -> trascina lo zip
echo.
pause