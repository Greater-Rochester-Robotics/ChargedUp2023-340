cd .\src
start /min ..\bin\pynetworktables2js.exe --team 340 --port 8000

start "" "%programfiles%\Google\Chrome\Application\chrome.exe" --app="http://localhost:8000" --window-position=1920,0 --kiosk --user-data-dir="%HOMEPATH%/dashboard0"
start "" "%programfiles%\Google\Chrome\Application\chrome.exe" --app="http://localhost:8000/joel.html" --window-position=-1920,0 --kiosk --user-data-dir="%HOMEPATH%/dashboard1"
