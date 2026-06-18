@echo off
REM ===========================================================================
REM  Blocking Focus (BF) firmware - derleme / flash yardimcisi
REM
REM  Kullanim (bu .bat esp32\BF klasorundedir; cift tiklanabilir):
REM    bf_build.bat              : sadece derle
REM    bf_build.bat COM16        : derle + COM16'ya flash + seri monitor
REM    bf_build.bat COM16 nomon  : derle + COM16'ya flash (monitor yok)
REM    bf_build.bat clean        : tam temizlik (fullclean) + derle
REM
REM  ONEMLI - flash etmeden once:
REM    SKAPP'in bu COM portuna baglantisini kes (Cihazlarim -> baglantiyi kes)
REM    ya da SKAPP'i kapat. Aksi halde port mesgul olur ve esptool
REM    "could not open port" / "Access is denied" hatasi verir.
REM ===========================================================================
setlocal

REM --- Bu makinedeki ESP-IDF kurulumu. Baska makinede bu iki satiri guncelle.
set "IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.5.2"
set "IDF_TOOLS_PATH=C:\Espressif"

REM --- Proje klasoru = bu .bat'in bulundugu klasor
set "BF_DIR=%~dp0"
if "%BF_DIR:~-1%"=="\" set "BF_DIR=%BF_DIR:~0,-1%"

REM --- Arg ayikla: arg1 = "clean" | COM portu | bos ; arg2 = "nomon"
REM     (Eski surumdeki findstr regex'i COM portunu hic yakalamiyordu, bu
REM      yuzden COM verince bile flash atlanip yalniz build oluyordu.)
set "DO_CLEAN="
set "PORT="
set "WANT_MON=1"
if /I "%~1"=="clean" (
    set "DO_CLEAN=1"
) else (
    if not "%~1"=="" set "PORT=%~1"
)
if /I "%~2"=="nomon" set "WANT_MON="

if not exist "%IDF_PATH%\export.bat" (
    echo [HATA] ESP-IDF bulunamadi: %IDF_PATH%
    echo Bu .bat icindeki IDF_PATH satirini kendi IDF yolunuza gore guncelle.
    goto :end
)

echo === ESP-IDF ortami hazirlaniyor ===
call "%IDF_PATH%\export.bat"

if defined DO_CLEAN (
    echo === fullclean ===
    python "%IDF_PATH%\tools\idf.py" -C "%BF_DIR%" fullclean
)

set "FWVER=?"
if exist "%BF_DIR%\version.txt" set /p FWVER=<"%BF_DIR%\version.txt"

echo === Derleniyor: Blocking Focus firmware v%FWVER% ===
python "%IDF_PATH%\tools\idf.py" -C "%BF_DIR%" build
if errorlevel 1 (
    echo [HATA] Derleme basarisiz.
    goto :end
)

echo.
echo === Derleme tamam (v%FWVER%) ===
echo Bin: %BF_DIR%\build\blocking_focus.bin

if not defined PORT goto :end

echo.
echo === %PORT% portuna flash ===
echo NOT: SKAPP bu porta bagliysa once baglantiyi kes (yoksa "Access is denied").
if defined WANT_MON (
    echo Monitor acilacak ^(cikis: Ctrl+]^)
    python "%IDF_PATH%\tools\idf.py" -C "%BF_DIR%" -p %PORT% flash monitor
) else (
    python "%IDF_PATH%\tools\idf.py" -C "%BF_DIR%" -p %PORT% flash
)
if errorlevel 1 (
    echo.
    echo [HATA] Flash basarisiz. En olasi neden: COM portu mesgul.
    echo   1. SKAPP'i kapat ya da Cihazlarim'dan bu cihaza baglantiyi kes
    echo   2. Baska seri monitor / PuTTY / idf.py monitor acik mi
    echo   3. Dogru port mu: Aygit Yoneticisi - Baglanti noktalari (COM ve LPT)
)

:end
endlocal
echo.
pause
