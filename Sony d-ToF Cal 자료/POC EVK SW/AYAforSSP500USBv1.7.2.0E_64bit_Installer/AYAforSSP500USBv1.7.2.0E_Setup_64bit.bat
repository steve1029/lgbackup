@echo off
setlocal enabledelayedexpansion

set HERE=%~dp0

for /f %%A in ('wmic os get locale') do (
  set line=%%A
  set cut=!line:~1!
  if defined cut (
    echo ">>" %%A
    set lcid=%%A
  )
)
echo %lcid%

if "%lcid%"=="0409" (
  set transforms= TRANSFORMS=":en-US.mst"
) else if "%lcid%"=="0411" (
  set transforms= TRANSFORMS=":ja-JP.mst"
) else (
  set transforms=
)
echo %transforms%

if "%PROCESSOR_ARCHITECTURE%" EQU "AMD64" (
  start msiexec /i %HERE%\Package\AYAforSSP500USBv1.7.2.0E_x64.msi %transforms%
) else (
  ECHO MsgBox "64 bit OS is required to install this product." ,vbCritical,"AYAforSSP500(x64)"  > msg.vbs
  cscript.exe msg.vbs > NUL
  DEL msg.vbs
)
