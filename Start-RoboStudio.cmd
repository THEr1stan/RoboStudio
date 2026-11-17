@echo off
cd /d "%~dp0"
if not exist "dist\RoboStudio-0.8.0-win-x64\RoboStudio.exe" (
  echo Desktop app not built. See docs/DEVELOPMENT.md for the Windows build command.
  pause
  exit /b 1
)
start "" "%~dp0dist\RoboStudio-0.8.0-win-x64\RoboStudio.exe"
