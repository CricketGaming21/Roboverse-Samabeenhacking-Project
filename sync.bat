@echo off
cd /d C:\Users\markk\Desktop\px4-finals
git add -A
git commit -m "sync: windows update"
git push origin finals
echo.
echo Done!
pause