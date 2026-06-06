@echo off
cd /d C:\Users\markk\Desktop\px4-finals
echo Syncing to GitHub...
git add -A
git commit -m "sync: windows update %date% %time%"
git push origin finals
echo.
echo Done! Changes pushed to GitHub.
pause