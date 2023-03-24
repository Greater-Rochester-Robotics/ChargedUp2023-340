cd .\src
start /min ..\bin\pynetworktables2js.exe --team 340 --port 8000

start "" "%programfiles(x86)%\Google\Chrome\Application\chrome.exe" --app="http://localhost:8000" --window-position=1920,0 --start-fullscreen --user-data-dir="%HOMEPATH%/dashboard0"
start "" "%programfiles(x86)%\Google\Chrome\Application\chrome.exe" --app="http://localhost:8000/joel.html" --window-position=0,0 --window-size=1920,850 --user-data-dir="%HOMEPATH%/dashboard1"
