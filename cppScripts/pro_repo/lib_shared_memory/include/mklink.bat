@echo off
%~d0
cd %~dp0
set srcDir=..\..\..\inc\lib_os_adapter
set dstDir=.
for %%i in ("%srcDir%\*.h") do ( mklink "%dstDir%\%%~ni%%~xi"  "%srcDir%\%%~ni%%~xi")

Pause 