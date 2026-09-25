@echo off
title Aggiornamento Pacchetto Portatile ZIP
cd /d "%~dp0"
echo Creazione dell'archivio DigitalTwin6DOF_Portable.zip in corso...
powershell -NoProfile -Command "$files = @('DigitalTwin6DOF.exe','Program.cs','AVVIA_SIMULATORE.bat','LEGGIMI_GUIDA.txt','index.html','ik_simulator_v29.html','ik_simulator_v28.html','base_finita.stl','braccio_1.stl','braccio_2.stl','braccio_fine.stl','collegamento_giunti_braccio_1.stl','collegamento_giunti_braccio_2.stl','collegamento_giunti_braccio_3.stl','coperchio_base_rotante.stl','robot6dof_stl_config.json','robot6dof_saved_poses.json','robot6dof_animations.json','libs'); Compress-Archive -Path $files -DestinationPath 'DigitalTwin6DOF_Portable.zip' -Force"
echo.
echo ========================================================
echo   DigitalTwin6DOF_Portable.zip creato con successo!
echo ========================================================
pause
