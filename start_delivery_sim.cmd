@echo off
cd /d "%~dp0"
python -m delivery_sim
if errorlevel 1 (
  echo.
  echo Simulator could not start. Confirm that Python 3.10 or newer is installed.
  pause
)
