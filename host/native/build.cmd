@echo off
setlocal
if not defined VSCMD_ARG_TGT_ARCH (
  for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do call "%%i\VC\Auxiliary\Build\vcvars64.bat"
)
pushd "%~dp0"
if not exist "..\..\artifacts\native" mkdir "..\..\artifacts\native"
cl /nologo /O2 /std:c++17 /EHsc /W4 /wd4324 /utf-8 s31_rx.cpp /Fo"..\..\artifacts\native\s31_rx.obj" /Fe"..\..\artifacts\native\s31_rx.exe" ws2_32.lib avrt.lib
set result=%errorlevel%
popd
exit /b %result%
