Set WshShell = CreateObject("WScript.Shell")
WshShell.Run "cmd /c cd /d C:\Users\markk\Desktop\px4-finals && git pull origin finals --quiet", 0, False
