@echo off
cd /d "%~dp0"
echo ==================================================
echo Lab 3 - SFML setup and Visual Studio 2022 startup
echo ==================================================
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup_sfml.ps1"
if errorlevel 1 (
  echo.
  echo Failed to get SFML. See instructions above.
  pause
  exit /b 1
)
echo SFML ready. Opening the solution...
start "" "%~dp0Lab3_SFML.sln"
if errorlevel 1 (echo Open Lab3_SFML.sln manually. & pause)
