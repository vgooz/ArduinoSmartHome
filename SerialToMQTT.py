#!/usr/bin/python
#
#simple app to read string from serial port
#and publish via MQTT
#

import time
import serial
import paho.mqtt.client as paho
import os

serialport = '/dev/ttyUSB0'
broker = "192.168.1.100"
port = 1883

try:
    while not os.path.exists(serialport):
        print("Serial Port %s is not found. Next attempt in 5 sec." % serialport)
        time.sleep(5)
except (KeyboardInterrupt):
    print ("Interrupt received")
    exit(0)

try:
    print ("Connecting to Serial... ", serialport)
    ser = serial.Serial(serialport, 115200, timeout=60)
    time.sleep(2)   # Wait while Arduino is not ready
except:
    print ("Failed open Serial Port")
    raise SystemExit

def cleanup():
    print ("Ending and cleaning up")
    mqttc.loop_stop()
    mqttc.disconnect()
    ser.close()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        paho.Client.connected_flag = True #set flag
        print("Connected to MQTT")
    else:
        print("Connection Failed with Code=", rc)

def on_message(client, userdata, message):
    mess = message.topic + " " + message.payload.decode("utf-8")
    print(mess)
    mess = mess + "\n"
    ser.write(mess.encode())
    ser.flush()

try:
    mypid = os.getpid()
    client_uniq = "ArduinoNano-"+str(mypid)
    mqttc = paho.Client(client_uniq)
    mqttc.username_pw_set("mqttusr", "mq123tt")

    mqttc.on_connect = on_connect
    mqttc.on_message = on_message
    
    paho.Client.connected_flag=False
    
    print("Connecting to MQTT", broker, end='', flush=True)
    #connect to broker
    mqttc.loop_start()
    mqttc.connect(broker, 1883, 60)
    while not paho.Client.connected_flag: #wait in loop
        print(".", end='', flush=True)
        time.sleep(1)

    print("Get Topics to subscribe")
    ser.write(b'get_topics\n')
    ser.flush()
    time.sleep(1)   # Wait while Arduino is not ready

    while True:
        line = ser.readline().rstrip().decode("utf-8")
        if line == "end":
            break
        print("Topic to Subscribe", line)
        mqttc.subscribe(line)

    #read data from serial and publish
    while True:
        line = ser.readline().rstrip().decode("utf-8")
        print(line)
        list = line.split(' ')
        mqttc.publish(list[0], list[1])

# handle list index error (i.e. assume no data received)
except (IndexError):
    print ("No data received within serial timeout period")
    cleanup()
# handle app closure
except (KeyboardInterrupt):
    print ("Interrupt received")
    cleanup()
except (RuntimeError):
    print ("uh-oh! time to die")
    cleanup()