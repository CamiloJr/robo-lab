@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title ROBO LAB - IA aplicada a Robotica

echo ==========================================================
echo    ROBO LAB - Simulador Interativo de Robotica com IA
echo ==========================================================
echo.
set "PYTHON="
where py >nul 2>&1
if not errorlevel 1 (
  for %%V in (3.14 3.13 3.12 3.11 3.10) do (
    if not defined PYTHON (
      py -%%V -c "import struct; assert struct.calcsize('P') == 8" >nul 2>&1
      if not errorlevel 1 set "PYTHON=py -%%V"
    )
  )
)
if not defined PYTHON (
  where python >nul 2>&1
  if not errorlevel 1 set "PYTHON=python"
)
if not defined PYTHON (
  echo ERRO: nao foi encontrado Python de 64 bits.
  echo Instale Python 3.14 de 64 bits e execute novamente.
  pause
  exit /b 1
)
%PYTHON% -c "import sys,struct; assert sys.version_info[:2] in ((3,14),(3,13),(3,12),(3,11),(3,10)) and struct.calcsize('P') == 8" >nul 2>&1
if errorlevel 1 (
  echo ERRO: Python incompativel. Use Python 3.10 a 3.14 de 64 bits.
  pause
  exit /b 1
)
echo Python encontrado:
%PYTHON% --version
if not exist ".venv\Scripts\python.exe" (
  echo [1/3] Criando ambiente virtual...
  %PYTHON% -m venv .venv
  if errorlevel 1 goto ERRO
)
echo [2/3] Verificando pygame-ce...
".venv\Scripts\python.exe" -c "import pygame" >nul 2>&1
if errorlevel 1 (
  ".venv\Scripts\python.exe" -m pip install --upgrade pip
  if errorlevel 1 goto ERRO
  ".venv\Scripts\python.exe" -m pip install -r requirements.txt
  if errorlevel 1 goto ERRO
)
echo [3/3] Iniciando simulador...
".venv\Scripts\python.exe" robotica_interativa.py
if errorlevel 1 goto ERRO
exit /b 0
:ERRO
echo.
echo Ocorreu uma falha. Confira a saida acima.
echo Caso o simulador tenha aberto e encerrado com erro, envie robotica_erros.log.
pause
exit /b 1
