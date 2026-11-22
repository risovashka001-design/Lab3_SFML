@echo off
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup_sfml.ps1"
if errorlevel 1 (
  echo.
  echo ERROR: Could not install SFML. See the error above.
  pause
  exit /b 1
)
echo.
echo SFML ready. Open Lab3_SFML.sln.
pause
