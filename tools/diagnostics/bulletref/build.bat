@echo off
REM  Build Bullet 2.75 core (LinearMath + BulletCollision + BulletDynamics) and bulletref.exe
REM  NOTE: ASCII-only on purpose. cmd.exe reads .bat in the OEM codepage (CP932 here), so UTF-8
REM        Japanese comments corrupt the parser. Japanese notes live in README_bulletref.md.
REM  cmake is NOT used: bullet 2.75's CMakeLists says cmake_minimum_required(2.4), which current
REM  cmake refuses. Only the 3 core libs are needed, so cl.exe is fed directly.
REM  113 sources exceed the 8191-char command line, so a response file is used.
REM  Gotcha: cl does NOT accept quotes around a response file. @"path" -> D8003. Use @path.
setlocal
REM  Override with VCVARS_PATH if Visual Studio lives elsewhere.
if not defined VCVARS_PATH set VCVARS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat
set VCVARS="%VCVARS_PATH%"
if not exist %VCVARS% (echo [build] vcvars64.bat not found & exit /b 1)
call %VCVARS% >nul
if errorlevel 1 (echo [build] vcvars64 failed & exit /b 1)

set HERE=%~dp0
set HERE=%HERE:~0,-1%
REM  Bullet 2.75 sources. Override with BULLET_SRC; default assumes a sibling of the repo.
if not defined BULLET_SRC set BULLET_SRC=%HERE%\..\..\..\..\bullet-reference\bullet-2.75\src
set BULLET=%BULLET_SRC%
if not exist "%BULLET%" (echo [build] bullet sources not found: %BULLET% ^(set BULLET_SRC^) & exit /b 1)
set OBJDIR=%HERE%\obj
if not exist "%OBJDIR%" mkdir "%OBJDIR%"

set WARN=/wd4267 /wd4244 /wd4305 /wd4996 /wd4819 /wd4311 /wd4302 /wd4312
set CFLAGS=/nologo /c /O2 /MT /EHsc /fp:strict /I%BULLET% /I%HERE% /D_CRT_SECURE_NO_WARNINGS %WARN%

if "%1"=="exeonly" goto exe
if exist "%OBJDIR%\bullet275.lib" if "%1"=="" goto haveLib

echo [build] collecting bullet core sources ...
pushd "%BULLET%"
if exist "%HERE%\lib.rsp" del "%HERE%\lib.rsp"
for /f "delims=" %%f in ('dir /s /b LinearMath\*.cpp BulletCollision\*.cpp BulletDynamics\*.cpp ^| findstr /v /i "ibmsdk"') do echo "%%f">>"%HERE%\lib.rsp"
popd
for /f %%c in ('find /c /v "" ^< "%HERE%\lib.rsp"') do echo [build]   %%c files

echo [build] compiling bullet 2.75 core (takes a few minutes) ...
cl %CFLAGS% /Fo%OBJDIR%\ @%HERE%\lib.rsp > "%HERE%\build_lib.log" 2>&1
if errorlevel 1 (echo [build] *** bullet core compile FAILED - see build_lib.log & exit /b 1)
lib /nologo /OUT:"%OBJDIR%\bullet275.lib" "%OBJDIR%\*.obj" >nul
if errorlevel 1 (echo [build] *** lib FAILED & exit /b 1)
echo [build] bullet275.lib OK
goto exe

:haveLib
echo [build] bullet275.lib already present (pass "rebuild" to force)

:exe
if not exist "%HERE%\bulletref.cpp" (echo [build] bulletref.cpp not present yet - library only & exit /b 0)
echo [build] compiling bulletref ...
REM /utf-8 : bulletref.cpp contains non-ASCII comment characters. Without it cl reads the
REM          file as CP932; some UTF-8 sequences decode with a trailing 0x5C (backslash),
REM          which splices the next line into a // comment and yields bogus errors.
cl /nologo /O2 /MT /EHsc /fp:strict /utf-8 /I%BULLET% /D_CRT_SECURE_NO_WARNINGS %WARN% "%HERE%\bulletref.cpp" /Fo"%HERE%\bulletref.obj" /Fe"%HERE%\bulletref.exe" /link "%OBJDIR%\bullet275.lib" > "%HERE%\build_exe.log" 2>&1
if errorlevel 1 (echo [build] *** bulletref build FAILED - see build_exe.log & exit /b 1)
echo [build] OK  -^>  %HERE%\bulletref.exe
endlocal
