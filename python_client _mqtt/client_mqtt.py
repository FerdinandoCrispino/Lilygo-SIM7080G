# -*- coding: utf-8 -*-
import os, sys
import json
import logging
import time

from typing import List, Dict, Any
import pandas as pd
import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes

# Get a logger instance
logger = logging.getLogger(__name__)

MAX_TENTATIVAS = 10
CONFIG_FILE = 'config_csa.json'
csa2048 = True

mytransport = 'tcp'                     #'websockets' # or 'tcp'
arquivo_excel = 'mensagens_mqtt.xlsx'   # Nome do arquivo Excel
arquivo_csv = 'mensagens_mqtt.csv'             # Nome do arquivo CSV
arquivo_json = 'mensagens_mqtt.json'         # Nome do arquivo JSON



def load_config(config_path: str = CONFIG_FILE, defaults: Dict[str, Any] = None) -> Dict[str, Any]:
    """
    Carrega configuração do arquivo 'config.json' e retorna um dicionário
    com chaves padrão: TOPICOS (lista), USERNAME, PASSWORD, CLIENT_ID, BROKER, PORT
    """
    if defaults is None:
        defaults = {
            "TOPICOS": ["sensor_01", "sensor_03"],
            "USERNAME": "EeObn5ov3lPkkzQUhQ",
            "PASSWORD": "QIKlRwWCtwPCnHN6U1",
            "CLIENT_ID": "client_receive",
            "BROKER": "crystalmq.bevywise.com",
            "PORT": 1883,
        }

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = json.load(f) or {}
    except FileNotFoundError:
        logger.warning("Arquivo %s não encontrado. Usando configuração padrão.", config_path)
        cfg = {}
    except json.JSONDecodeError as e:
        logger.error("Erro ao decodificar %s: %s. Usando configuração padrão.", config_path, e)
        cfg = {}

    # Normalize keys e preencha com defaults
    result = defaults.copy()
    # aceitar TOPICO, TOPICOS ou topics
    topics_raw = cfg.get('TOPICOS') or cfg.get('TOPICO') or cfg.get('topics') or defaults['TOPICOS']

    # Normaliza tópicos para lista de strings
    if isinstance(topics_raw, str):
        topics = [topics_raw]
    elif isinstance(topics_raw, (list, tuple)):
        topics = [str(t) for t in topics_raw]
    else:
        topics = [str(topics_raw)]

    result.update({
        "TOPICOS": topics,
        "USERNAME": cfg.get('USERNAME', defaults['USERNAME']),
        "PASSWORD": cfg.get('PASSWORD', defaults['PASSWORD']),
        "CLIENT_ID": cfg.get('CLIENT_ID', defaults['CLIENT_ID']),
        "BROKER": cfg.get('BROKER', defaults['BROKER']),
        "PORT": cfg.get('PORT', defaults['PORT']),
    })

    return result

#def load_topics(config_path: str = CONFIG_FILE) -> List[str]:
#    """Retorna apenas a lista de tópicos usando load_config."""
#    return load_config(config_path)['TOPICOS']

cfg = load_config(CONFIG_FILE)

# Configurações do broker com autenticação
#BROKER = "crystalmq.bevywise.com"   	# Substitua pelo IP ou endereço do broker
#PORT = 1883                    		    # Porta padrão (ou 8883 para TLS)
#TOPICO = load_topics()                  # nome do Topico cadastrado no Broker MQTT
#USERNAME = "EeObn5ov3lPkkzQUhQ"         #"EeObn5ov3lmm7b1zZo"
#PASSWORD = "QIKlRwWCtwPCnHN6U1"         #"QJeEn6EawcJ8gSoQCF"
#CLIENT_ID  = "client_receive"           #"teste1"

#USERNAME = "EeObn5ov3lIC5xGbY2"         #"EeObn5ov3lmm7b1zZo"
#PASSWORD = "xaPS1cbCxj1kKadb6c"         #"QJeEn6EawcJ8gSoQCF"
#CLIENT_ID  = "client_receive_ferd"           #"teste1"

# caixa sueca - Dados para o cliente que vai receber leituras
#TOPICO = load_topics()
#CLIENT_ID = "A7672_Receive"
#USERNAME =  "URCnsl7Oz40wta4zEo"
#PASSWORD = "WhI4doaY6CIaYXCJS1"

BROKER = cfg.get("BROKER")
PORT = cfg.get("PORT")
TOPICO = cfg.get("TOPICOS")  # lista de tópicos
USERNAME = cfg.get("USERNAME")
PASSWORD = cfg.get("PASSWORD")
CLIENT_ID = cfg.get("CLIENT_ID")




def load_topics(config_path: str = CONFIG_FILE, default=None):
    """Carrega lista de tópicos de um arquivo JSON. Retorna lista de strings."""
    if default is None:
        default = ["sensor_01", "sensor_03"]
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
    except FileNotFoundError:
        logger.warning("Arquivo %s não encontrado. Usando TOPICO padrão.", config_path)
        return default
    except json.JSONDecodeError as e:
        logger.error("Erro ao decodificar %s: %s. Usando TOPICO padrão.", config_path, e)
        return default
    # aceita chaves TOPICO, TOPICOS ou topics
    topics = cfg.get('TOPICO') or cfg.get('TOPICOS') or cfg.get('topics')
    if topics is None:
        logger.warning("Chave de tópicos não encontrada em %s. Usando padrão.", config_path)
        return default
    if isinstance(topics, str):
        return [topics]
    if isinstance(topics, (list, tuple)):
        return [str(t) for t in topics]
    # tenta converter outros tipos para string
    return [str(topics)]

def get_data_folder():
    # path of your data in same folder of main .py or added using --add-data
    if getattr(sys, 'frozen', False):
        data_folder_path = os.path.dirname(sys.executable)
    else:
        data_folder_path = os.path.dirname(
            os.path.abspath(sys.modules['__main__'].__file__)
        )
    return data_folder_path

def salvar_dados(msg_dict, topic):
    global arquivo_csv, arquivo_excel
    if isinstance(msg_dict, str):
        if csa2048:
            if  topic == TOPICO[0]:
                arquivo_csv = 'mensagens_mqtt_csa2048_dados.csv'
                arquivo_excel = 'mensagens_mqtt_csa2048_dados.xlsx'
            else:
                arquivo_csv = 'mensagens_mqtt_csa2048_diag.csv'
                arquivo_excel = 'mensagens_mqtt_csa2048_diag.xlsx'

        # Tenta converter a mensagem em um dicionário
        try:
            msg_dict = json.loads(msg_dict)
        except json.JSONDecodeError as e:
            print(f"❌ Erro ao decodificar JSON: {e}")
            logger.error(f"Erro ao decodificar JSON: {e}")
            return

        # Adiciona os novos dados ao DataFrame
        novo_dado = pd.DataFrame([msg_dict])
        novo_dado['sensor'] = topic
        novo_dado.drop_duplicates(inplace=True)
        # salva arquivo csv
        try:
            novo_dado.to_csv(arquivo_csv, mode='a', index=False, header=False)
        except Exception as e:
            logger.error("Erro ao gravar CSV: %s", e)

        print(f"📊 Dados salvos no arquivo {arquivo_csv}")

        #if 'lat' in novo_dado.columns:
        #    novo_dado['lat'] = novo_dado['lat'].astype(float)
        #if 'lon' in novo_dado.columns:
        #    novo_dado['lon'] = novo_dado['lon'].astype(float)


        # Salva o DataFrame no arquivo Excel
        try:
            if os.path.exists(arquivo_excel):
                df = pd.read_excel(arquivo_excel, dtype=str, engine='openpyxl')
            else:
                df = pd.DataFrame()

            df = pd.concat([df, novo_dado], ignore_index=True)
            df.drop_duplicates(inplace=True)

            df.to_excel(arquivo_excel, float_format='%.7f', index=False, engine='openpyxl')
        except Exception as e:
            logger.error("Erro ao gravar Excel: %s", e)

        print(f"📊 Dados salvos no arquivo {arquivo_excel}")


"""
def salvar_dados_excel(msg_dict, topic):

    # print(msg_dict)
    # Verifica se o arquivo já existe
    if os.path.exists(arquivo_excel):
        # Carrega o arquivo existente
        df = pd.read_excel(arquivo_excel,  dtype=str, engine='openpyxl')
    else:
        # Cria um novo DataFrame se o arquivo não existir
        df = pd.DataFrame()

    # Tenta converter a mensagem em um dicionário
    if isinstance(msg_dict, str):
        try:
            msg_dict = json.loads(msg_dict)
        except json.JSONDecodeError as e:
            print(f"❌ Erro ao decodificar JSON: {e}")
            return

    # Adiciona os novos dados ao DataFrame
    novo_dado = pd.DataFrame([msg_dict])
    novo_dado['sensor'] = topic
    df = pd.concat([df, novo_dado], ignore_index=True)
    df.drop_duplicates(inplace=True)
    if 'lat' in df.columns:
        df['lat'] = df['lat'].astype(float)
    if 'lon' in df.columns:
        df['lon'] = df['lon'].astype(float)

    # Salva o DataFrame no arquivo Excel
    df.to_excel(arquivo_excel, float_format='%.5f', index=False,  engine='openpyxl')
    #df.to_csv(arquivo_csv, index=False, encoding='utf-8')
    novo_dado.to_csv(arquivo_csv, mode='a', index=False, header=None)

    print(f"📊 Dados salvos no arquivo {arquivo_excel}")
"""

# Callback quando conecta ao broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ Conectado com sucesso!")
        #client.subscribe("teste/eco")
        #client.publish("teste/eco", "Mensagem enviada e recebida!")
        # Subscribe to multiple topics after successful connection
        # The list contains tuples: (topic, QoS)
        #client.subscribe([("topic/one", 1), ("topic/two", 1)])

        # client.subscribe(TOPICO, qos=1)
        for tp in TOPICO:
            client.subscribe(tp, qos=1)
            print(f'SUBSCRIBED: {tp}')

    else:
        print("❌ Falha na conexão. Código de erro:", rc)

# Callback quando uma mensagem é recebida
def on_message(client, userdata, msg):
    print(f"📩 Tópico: {msg.topic} | Mensagem: {msg.payload.decode()}")
    print('\n{} RCVD Topic:{}, Message:{}, QoS:{} PktId:{}'.format(client._client_id.decode(), msg.topic, msg.payload,
                                                                   msg.qos, msg.mid))

    # salvar_dados_excel(msg.payload.decode(), msg.topic)
    salvar_dados(msg.payload.decode('utf-8'), msg.topic)

def on_unsubscribe(client, userdata, mid):
    print('\n{} UNSUBSCRIBED'.format(client._client_id.decode()))
    for tp in TOPICO:
        client.unsubscribe(tp)

def on_subscribe(client, userdata, mid, granted_qos):
    print('\n{} SUBSCRIBED with QoS:{}'.format(client._client_id.decode(),granted_qos))

version = '3' # '3' or '5'

# Criação do cliente
if version == '5':
    client = mqtt.Client(client_id=CLIENT_ID,
                         transport=mytransport,
                         protocol=mqtt.MQTTv5)
if version == '3':
    client = mqtt.Client(CLIENT_ID, transport=mytransport, protocol=mqtt.MQTTv311, clean_session=False)

# Define usuário e senha para autenticação
client.username_pw_set(username=USERNAME, password=PASSWORD)

# Define callbacks
client.on_connect = on_connect
client.on_message = on_message
client.on_subscribe = on_subscribe

# cria Properties padrão; ajustar se preciso (ex.: SessionExpiryInterval, UserProperties, ...)
properties = Properties(PacketTypes.CONNECT)
for tentativa in range(1, MAX_TENTATIVAS + 1):
    try:
        if version == '5':
            client.connect(BROKER,
                           port=PORT,
                           clean_start=mqtt.MQTT_CLEAN_START_FIRST_ONLY,
                           properties=properties,
                           keepalive=60)
        elif version == '3':
            client.connect(BROKER, port=PORT, keepalive=60)
        print(f"✅ Conectado na tentativa {tentativa}")
        break
    except Exception as e:
        print(f"❌ Falha na conexão ({tentativa}/10): {e}")
        if tentativa < MAX_TENTATIVAS:
            print("⏳ Aguardando 1 minuto para nova tentativa...")
            time.sleep(60)
        else:
            print("🚫 Número máximo de tentativas atingido. Encerrando.")
            exit(1)


# Conecta ao broker
# client.connect(BROKER, PORT, keepalive=60)
client.loop_forever()


# para criar um executável use:
# pyinstaller --onefile --name client_mqtt client_mqtt.py
# pyinstaller --onefile --name client_mqtt_csa2048 client_mqtt.py

# pip freeze > requirements.txt

