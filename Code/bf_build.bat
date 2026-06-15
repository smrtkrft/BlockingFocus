@echo off
REM ===========================================================================
REM  Blocking Focus (BF) firmware - derleme / flash yardimcisi
REM  Firmware surumu: 0.3.1   (kaynak: main.c SK_FW_VERSION + version.txt)
REM
REM  Kullanim (bu .bat esp32\BF klasorundedir):
REM    bf_build.bat              : sadece derle
REM    bf_build.bat COM7         : derle + COM7'ye flash + seri monitor ac
REM    bf_build.bat COM7 nomon   : derle + COM7'ye flash (monitor yok)
REM    bf_build.bat clean        : tam temizlik (fullclean) + derle
REM
REM  Cikti: build\blocking_focus.bin
REM  Monitor'den cikis: Ctrl+]
REM ===========================================================================
setlocal

REM --- Bu makinedeki ESP-IDF kurulumu. Baska makinede bu iki satiri guncelleyin.
set "IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.5.2"
set "IDF_TOOLS_PATH=C:\Espressif"

REM --- Proje klasoru = bu .bat'in bulundugu klasor
set "BF_DIR=%~dp0"
if "%BF_DIR:~-1%"=="\" set "BF_DIR=%BF_DIR:~0,-1%"

if not exist "%IDF_PATH%\export.bat" (
    echo [HATA] ESP-IDF bulunamadi: %IDF_PATH%
    echo Bu .bat icindeki IDF_PATH satirini kendi IDF yolunuza gore guncelleyin.
    exit /b 1
)

echo === ESP-IDF ortami hazirlaniyor ===
call "%IDF_PATH%\export.bat"

if /I "%~1"=="clean" (
    echo === fullclean ===
    python "%IDF_PATH%\tools\idf.py" -C "%BF_DIR%" fullclean
)

echo === Derleniyor: Blocking Focus v0.3.1 ===
python "%IDF_PATH%\tools\idf.py" -C "%BF_DIR%" build
if errorlevel 1 (
    echo [HATA] Derleme basarisiz.
    exit /b 1
)

echo.
echo === Derleme tamam ===
echo Bin: %BF_DIR%\build\blocking_focus.bin

REM --- Ilk arg bir COM portu ise flash et ---
echo.%~1| findstr /I /R "^.COM[0-9]" >nul
if not errorlevel 1 (
    if /I "%~2"=="nomon" (
        echo === %~1 portuna flash ===
        python "%IDF_PATH%\tools\idf.py" -C "%BF_DIR%" -p %~1 flash
    ) else (
        echo === %~1 portuna flash + monitor ^(cikis: Ctrl+]^) ===
        python "%IDF_PATH%\tools\idf.py" -C "%BF_DIR%" -p %~1 flash monitor
    )
)

endlocal
