
@echo off

if "%~5" == "" goto Usage
if "%~6" == "" goto KeepGoing
goto Usage
:KeepGoing

REM User build script for Keil ARMCC compiler.
REM Will automatically sign executable and build
REM packets for use with SCP protocol.
REM Add this batch file to Target->Options->User
REM "Run User Programs After Build/Rebuild"
REM Parameters are:
REM 1 - Path to fromelf.exe ($K)
REM 2 - Build directory ($L)
REM 3 - Linker output file (.axf, !L)
REM 4 - Output file with no extension (.axf, @L)
REM 5 - CPU Target Name ($D, e.g. MAX32550)
echo Path to fromelf.exe: %~1
echo Path to axf file (linker output) dir: %~2
echo Linker output file name: %~3
echo Project name: %~4
echo Device name: %~5

SET KEIL_PATH=%~1
SET BUILD_PATH=%~2
SET LINKER_FILE=%~3
SET PROJ_NAME=%~4
SET DEVICE_NAME=%5

SET BUILD_PATH_SHORT=%~s2
SET PROJ_NAME_SHORT=%~s4

SET CA_SIGN_BUILD="%~dp0ca_sign_build\ca_sign_build.exe"
SET SESSION_BUILD="%~dp0session_build\session_build.exe"
SET HEXCONVERTER="%~dp0hexconverter\hexconverter.exe"
SET KEY_FILE="%~dp0ca_sign_build\crk_ecdsa_MAX3255X_test.key"

echo Removing old build files
IF exist "%BUILD_PATH%scp_packets" (
IF exist "%BUILD_PATH%scp_packets\scp_packets*" del "%BUILD_PATH%scp_packets\scp_packets*" /Q
IF exist "%BUILD_PATH%scp_packets\packet.list" del "%BUILD_PATH%scp_packets\packet.list" /Q
rmdir "%BUILD_PATH%scp_packets"
)
IF exist "%BUILD_PATH%%PROJ_NAME%.bin" del "%BUILD_PATH%%PROJ_NAME%.bin" /Q
IF exist "%BUILD_PATH%%PROJ_NAME%.sbin" del "%BUILD_PATH%%PROJ_NAME%.sbin" /Q
IF exist "%BUILD_PATH%signed.hex" del "%BUILD_PATH%signed.hex" /Q
IF exist "%BUILD_PATH%signed.s19" del "%BUILD_PATH%signed.s19" /Q

echo Extract raw binary from .axf file
"%KEIL_PATH%ARM\ARMCC\bin\fromelf.exe" --bin --output="%BUILD_PATH%%PROJ_NAME%.bin" "%LINKER_FILE%"
if errorlevel=1 goto Error

echo Sign binary file
IF %DEVICE_NAME% == MAX32550 (
echo MAX32550 Signing Tool Selected
%CA_SIGN_BUILD% version=01000003 arguments= load_address=10000000 jump_address=10000020 algo=ecdsa ecdsa_file=%KEY_FILE% ca="%BUILD_PATH%%PROJ_NAME%.bin" sca="%BUILD_PATH%%PROJ_NAME%.sbin" verbose=no
)
IF %DEVICE_NAME% == MAX32555 (
echo MAX32555 Signing Tool Selected
%CA_SIGN_BUILD% version=01010003 arguments= load_address=10000000 jump_address=10000020 algo=ecdsa ecdsa_file=%KEY_FILE% ca="%BUILD_PATH%%PROJ_NAME%.bin" sca="%BUILD_PATH%%PROJ_NAME%.sbin" verbose=no
)
if errorlevel=1 goto Error

echo Convert signed binary to hex for Keil loading
%HEXCONVERTER% -r bin -F hex -i "%BUILD_PATH%%PROJ_NAME%.sbin" -o "%BUILD_PATH%signed.hex" -e -3 -s 0x10000000
if errorlevel=1 goto Error

REM Generate signed packet stream for loading over SCP
echo Convert signed binary to packet stream for SCP loading.

IF exist "%BUILD_PATH%scp_packets" (
IF exist "%BUILD_PATH%scp_packets\scp_packets*" del "%BUILD_PATH%scp_packets\scp_packets*" /Q
IF exist "%BUILD_PATH%scp_packets\packet.list" del "%BUILD_PATH%scp_packets\packet.list" /Q
rmdir "%BUILD_PATH%scp_packets"
)
mkdir "%BUILD_PATH%scp_packets"

REM Convert signed binary to srecord for session_build.exe
%HEXCONVERTER% -r bin -F srec -i "%BUILD_PATH%%PROJ_NAME%.sbin" -o "%BUILD_PATH%signed.s19" -e -3 -s 0x10000000
if errorlevel=1 goto Error

echo write-file %BUILD_PATH_SHORT%signed.s19 > "%BUILD_PATH%sb_script.txt"
%SESSION_BUILD% pp=ECDSA session_mode=SCP_ANGELA_ECDSA ecdsa_file=%KEY_FILE% script_file="%BUILD_PATH%sb_script.txt" output_file="%BUILD_PATH%scp_packets\scp_packets.output" chunk_size=2074 verbose=no
if errorlevel=1 goto Error

REM Build packet list file
cd "%BUILD_PATH%"scp_packets
dir /b /on scp_packets*.packet > packet.list

echo Build Succeeded.
exit /B 0

:Error
echo Build Failed!
exit /B 1

:Usage
echo usage: signit param1 param2 param3 param4 param5
echo.
echo param1    Path to fromelf.exe
echo param2    Path to axf file (linker output) dir
echo param3    Linker output file name
echo param4    Project name
echo param5    Device name (MAX32550 or MAX32555)

exit /B 2
