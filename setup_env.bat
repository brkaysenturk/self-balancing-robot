@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"

rem ------------------------------------------------------------------
rem 0) Projeye ozel onbellek ayarlari.
rem ------------------------------------------------------------------
set "PYTHONDONTWRITEBYTECODE=1"
set "RUFF_NO_CACHE=true"
set "PIP_NO_CACHE_DIR=1"

rem ------------------------------------------------------------------
rem VS Code yerel ortam dosyasini yalnizca yoksa olustur.
rem ------------------------------------------------------------------
if not exist ".env" copy ".env.example" ".env" >nul
if not exist ".env" goto :env_failed

echo ==============================================================
echo    SELF BALANCING ROBOT // OTOMATIK ORTAM KURULUMU
echo ==============================================================
echo.

rem ------------------------------------------------------------------
rem 1) Python 3.12, 3.11 veya 3.10 surumunu bul.
rem ------------------------------------------------------------------
set "PY_CMD="
call :try_py 3.12
call :try_py 3.11
call :try_py 3.10
if not defined PY_CMD (
    python -c "import sys; raise SystemExit(0 if (3,10) <= sys.version_info[:2] < (3,13) else 1)" >nul 2>nul
    if not errorlevel 1 set "PY_CMD=python"
)

if not defined PY_CMD goto :no_python

echo [OK] Python bulundu:
%PY_CMD% -c "import sys; print('    ', sys.version.split()[0], sys.executable)"

rem ------------------------------------------------------------------
rem 2) Proje sanal ortamini olustur veya onar.
rem ------------------------------------------------------------------
set "VENV_PY=venv\Scripts\python.exe"
if not exist "%VENV_PY%" goto :build_venv

"%VENV_PY%" -c "import sys; raise SystemExit(0 if (3,10) <= sys.version_info[:2] < (3,13) else 1)" >nul 2>nul
if errorlevel 1 goto :broken_venv
echo [INFO] Mevcut venv kullanilabilir.
goto :have_venv

:broken_venv
echo [WARN] Mevcut venv bozuk veya desteklenmeyen Python surumune bagli; yeniden kuruluyor.

:build_venv
echo [1/3] Sanal ortam olusturuluyor (venv)...
if exist "venv" rmdir /s /q "venv"
%PY_CMD% -m venv venv
if errorlevel 1 goto :venv_failed
echo [OK] venv olusturuldu.

:have_venv
rem ------------------------------------------------------------------
rem 3) pip'i guncelle.
rem ------------------------------------------------------------------
echo.
echo [2/3] pip guncelleniyor...
"%VENV_PY%" -B -m pip install --upgrade pip
if errorlevel 1 goto :pip_failed

echo.
rem ------------------------------------------------------------------
rem 4) Proje ve test bagimliliklarini pyproject.toml dosyasindan kur.
rem ------------------------------------------------------------------
echo [3/3] Proje ve test bagimliliklari kuruluyor...
"%VENV_PY%" -B -m pip install -e ".[test]"
if errorlevel 1 goto :deps_failed

echo.
rem ------------------------------------------------------------------
rem 5) Paketlerin ice aktarilabildigini dogrula.
rem ------------------------------------------------------------------
echo [Dogrulama] Proje ve ana bagimliliklar kontrol ediliyor...
"%VENV_PY%" -B -c "import matplotlib, numpy, optuna, pytest, self_balancing_robot; print('[OK] numpy', numpy.__version__, '| matplotlib', matplotlib.__version__, '| optuna', optuna.__version__, '| pytest', pytest.__version__)"
if errorlevel 1 goto :verify_failed

echo.
echo ==============================================================
echo  [TAMAMLANDI] Proje ortami venv icinde hazir.
echo  Testler: venv\Scripts\python.exe -B -m pytest tests\ -q
echo  Lint:    venv\Scripts\python.exe -B -m ruff check self_balancing_robot tests analysis webots
echo ==============================================================
goto :done

:no_python
echo [HATA] Python 3.10 - 3.12 bulunamadi.
echo        https://www.python.org/downloads/ adresinden kurun ve
echo        kurulumda "Add python.exe to PATH" secenegini isaretleyin.
goto :fail

:venv_failed
echo [HATA] venv olusturulamadi.
goto :fail

:env_failed
echo [HATA] .env dosyasi .env.example dosyasindan olusturulamadi.
goto :fail

:pip_failed
echo [HATA] pip guncellenemedi. Internet baglantisini kontrol edin.
goto :fail

:deps_failed
echo [HATA] Proje bagimliliklari kurulamadi.
goto :fail

:verify_failed
echo [HATA] Kurulum dogrulanamadi.
goto :fail

:fail
if not defined CI pause
endlocal
exit /b 1

:done
if not defined CI pause
endlocal
exit /b 0

:try_py
rem Yuklu Python baslaticilarinda ilk bulunan surumu sec.
if defined PY_CMD exit /b 0
py -%~1 -c "import sys" >nul 2>nul
if not errorlevel 1 set "PY_CMD=py -%~1"
exit /b 0
