taskkill /IM obs64.exe /f
taskkill /IM putty.exe /f
taskkill /IM plink.exe /f


cd "C:\Users\Cody1\Desktop\2019 Vision"
start /b plink.exe -load COM14 < mountJevois.txt

sleep 10

taskkill /IM plink.exe /f

set "FROM_FILE=TapeDetectCody.py"
set "TO_FILE=D:\modules\Highlanders\TapeDetectCody\"
cp %FROM_FILE% %TO_FILE%

set "FROM_FILE=script.cfg"
set "TO_FILE=D:\modules\Highlanders\TapeDetectCody\"
cp %FROM_FILE% %TO_FILE%

set "FROM_FILE=initscript.cfg"
set "TO_FILE=D:\config\"
cp %FROM_FILE% %TO_FILE%

set "FROM_FILE=videomappings.cfg"
set "TO_FILE=D:\config\"
cp %FROM_FILE% %TO_FILE%

dir %TO_FILE%

sleep 5

start /b plink.exe -load COM14 < restartJevois.txt
taskkill /IM plink.exe /f

sleep 15

cd "C:\Users\Cody1\Desktop"
start /b putty.exe -load COM14

cd "C:\Program Files\obs-studio\bin\64bit"
start /b obs64.exe

