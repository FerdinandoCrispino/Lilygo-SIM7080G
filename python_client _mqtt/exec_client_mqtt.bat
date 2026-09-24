@echo off
REM Ativa o ambiente virtual
call venv\Scripts\activate

REM Executa o script Python
python client_mqtt.py

REM Desativa o ambiente virtual após a execução
deactivate