@echo off
setlocal
cd /d D:\test\Container_Total_Length_DLL

REM Prefer VS2013-era PCL deps if present; else use IPC path3 backup DLLs.
set DEP=D:\work\IPC_Station2\deploy_backup\container_total_length_20260819_0937
set PATH=%DEP%;D:\work\IPC_Station2;%PATH%

echo === PATH head ===
echo %DEP%

echo === Updating config.ini for field cloud ===
copy /Y config.ini config.ini.bak_demo >nul
powershell -NoProfile -Command ^
  "(Get-Content -Raw 'config.ini') -replace 'inputCloud\s*=\s*.*', 'inputCloud = ./Data/Field_Path3_Arm_All_20260911.ply' | Set-Content -NoNewline 'config.ini' -Encoding UTF8"

echo === config Input section ===
findstr /i /c:"inputCloud" /c:"crop" /c:"templateCloud" config.ini

echo === Copy missing PCL dlls next to exe if needed ===
if not exist x64\Release\pcl_common.dll copy /Y "%DEP%\pcl_*.dll" x64\Release\ >nul
if not exist x64\Release\pcl_io.dll (
  if exist D:\work\IPC_Station2\pcl_io.dll copy /Y D:\work\IPC_Station2\pcl_io.dll x64\Release\ >nul
)

echo === DLLs beside exe ===
dir /b x64\Release\*.dll

echo === Running demo ===
cd /d D:\test\Container_Total_Length_DLL
x64\Release\ContainerTotalLengthDemo.exe > D:\test\Container_Total_Length_DLL\demo_field_run.log 2>&1
set RC=%ERRORLEVEL%
echo EXIT_CODE=%RC%
type D:\test\Container_Total_Length_DLL\demo_field_run.log
echo EXIT_CODE=%RC% > D:\test\Container_Total_Length_DLL\demo_field_run.exit
exit /b %RC%
