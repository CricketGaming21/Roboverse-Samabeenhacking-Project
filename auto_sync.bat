@echo off
:loop
cd C:\Users\markk\Desktop\px4-finals
git pull origin finals --quiet
timeout /t 60 /nobreak >nul
goto loop