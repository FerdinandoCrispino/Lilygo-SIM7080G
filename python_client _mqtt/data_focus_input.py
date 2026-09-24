
import paho.mqtt.client
import sys
import time
import random
import string
import json
import signal
import os

# Configurações do broker com autenticação
BROKER = "crystalmq.bevywise.com"  	# Substitua pelo IP ou endereço do broker
PORT = 1883                    		# Porta padrão (ou 8883 para TLS)
TOPICO = "sensor_01"
USERNAME = "EeObn5ov3lmm7b1zZo"
PASSWORD = "QJeEn6EawcJ8gSoQCF"
CLIENT_ID  = "teste1"



def on_connect(client, userdata, flags, rc, properties):
    print('\n{} Connected'.format(client._client_id.decode()))


def on_message(client, userdata, message, properties):
    print('\n{} RCVD Topic:{}, Message:{}, QoS:{} PktId:{}'.format(client._client_id.decode(), message.topic,
                                                                   message.payload.decode(), message.qos, message.mid))


def on_publish(client, userdata, mid, properties):
    print('\n{} SENT PktID:{}'.format(client._client_id.decode(), mid))


def on_subscribe(client, userdata, mid, qos, properties):
    print('\n{} SUBSCRIBED with QoS:{}'.format(client._client_id.decode(), qos))


def on_unsubscribe(client, userdata, mid, properties):
    print('\n{} UNSUBSCRIBED'.format(client._client_id.decode()))


def on_disconnect(client, userdata, rc, properties):
    print('\n{} DISCONNECTED with rc:{}'.format(client._client_id.decode(), rc))
    client.loop_stop()

random_chars = ''.join(random.choices(string.ascii_letters + string.digits, k=10))
client_id = 'crystalmq_' + random_chars

client = paho.mqtt.client.Client(client_id, protocol=paho.mqtt.client.MQTTv5)

client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish
client.on_subscribe = on_subscribe
client.on_unsubscribe = on_unsubscribe
client.on_disconnect = on_disconnect

# Optional: Set authentication and other options
# client.username_pw_set("some username", "some password")
# client.will_set("willtopic", payload="Good bye CrystalMQ", qos=0, retain=False)
client.connect(BROKER, PORT, keepalive=40)

# Optional: Set MQTTv5 properties for connection
# connect_properties = paho.mqtt.properties.Properties(paho.mqtt.packettypes.PacketTypes.CONNECT)
# connect_properties.SessionExpiryInterval = 60
# connect_properties.MaximumPacketSize = 256
# connect_properties.TopicAliasMaximum = 10
# client.connect('public-mqtt-broker.bevywise.com', 1883, properties=connect_properties)

client.loop_start()
client.subscribe(TOPICO, qos=0)



