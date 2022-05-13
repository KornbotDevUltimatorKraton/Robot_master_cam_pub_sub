import threading
from xml.sax.handler import property_declaration_handler
import requests # Getting the micro controller data  
import socket
import json 
import pickle
import getpass 
import pandas as pd
import gpiozero 
from gpiozero import Robot, MCP3008
from gpiozero import PhaseEnableMotor # Getting the motor to working 
import face_recognition # Getting the face recognition to working 
import pyfirmata # Getting the pyfirmata for the librery of the serial communication between the hardware
from pyzbar import pyzbar 
from printrun.printcore import printcore
from printrun import gcoder
import serial 
import csv 
import re 
import os ,sys ,time 
import datetime # Getting date time data 
from itertools import count
import cv2,imutils
import math 
import numpy as np 
from pyzbar import pyzbar
import base64
import smbus 

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                                               #These function only able to enable from the singleboard computer 

#Support python version 3.7+ 3.8+ 
#Rpi fully support on the debian OS
#For the Jetson nano running on python version 3.6 not support 

# I2C servo motor board with the PCA9685
from board import SCL, SDA
import busio

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
try:
  pca = PCA9685(i2c_bus)
  pca.frequency = 50
except:
   print("No servo devices connect with the computer")
mem_sub_variable = [] # mem subscriber return variable 
mem_used_pins = {} # Collected the used pins on the list to avoid clash on the system hardware control 
Board_pins_sbc = {"Raspberry_pi_4":[{'label': '3.3V',      'type': 'Power'},
        {'label': '5V',        'type': 'Power'},
        {'label': 'BCM 2',     'type': 'IO'},
        {'label': '5V',        'type': 'Power'},
        {'label': 'BCM 3',     'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 4',     'type': 'IO'},
        {'label': 'BCM 14',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 15',    'type': 'IO'},
        {'label': 'BCM 17',    'type': 'IO'},
        {'label': 'BCM 18',    'type': 'IO'},
        {'label': 'BCM 27',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 22',    'type': 'IO'},
        {'label': 'BCM 23',    'type': 'IO'},
        {'label': '3.3V',      'type': 'Power'},
        {'label': 'BCM 24',    'type': 'IO'},
        {'label': 'BCM 10',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 9',     'type': 'IO'},
        {'label': 'BCM 25',    'type': 'IO'},
        {'label': 'BCM 11',    'type': 'IO'},
        {'label': 'BCM 8',     'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 7',     'type': 'IO'},
        {'label': 'BCM 0',     'type': 'IO'},
        {'label': 'BCM 1',     'type': 'IO'},
        {'label': 'BCM 5',     'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 6',     'type': 'IO'},
        {'label': 'BCM 12',    'type': 'IO'},
        {'label': 'BCM 13',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 19',    'type': 'IO'},
        {'label': 'BCM 16',    'type': 'IO'},
        {'label': 'BCM 26',    'type': 'IO'},
        {'label': 'BCM 20',    'type': 'IO'},
        {'label': 'GND',       'type': 'Ground'},
        {'label': 'BCM 21',    'type': 'IO'}]}

class Microcontroller_pins(object): # Choose one microcontroller to control the pheripheral devices 
               #Request the microcontroller data from the api link 
               def request_mcu(self,mcu_code):

                     r = requests.get("https://raw.githubusercontent.com/KornbotDevUltimatorKraton/mcusdata.github.io/main/"+mcu_code+".json")
                     status = r.status_code 
                     data = r.json()
                     Pins_label = {}
                     Pins_list = []
                     Sub_label = {}
                     Sub_label2 = {}
                     ref_sub = {}
                     #sprint(status,data)
                     get_pins = data.get('Mcu') # getting the data import from the mcu requests
                     #print(get_pins)
                     print("Package_infos")
                     #for r in range(0,len(get_pins)):       
                     #        print(list(get_pins)[r])
                     #print(get_pins.get("Pin"))
                     for pins in range(0,len(get_pins.get("Pin"))):
       
                               #print(list(get_pins.get("Pin"))[pins])
                               Sub_label['label'] = list(get_pins.get("Pin"))[pins].get("@Name")             
                               Sub_label2['type'] = list(get_pins.get("Pin"))[pins].get("@Type")
                               #print(Sub_label,Sub_label2)
                               if len(Sub_label) > 1: 
                                    del Sub_label[next(iter(Sub_label))]
                                    print(Sub_label) 
                               if len(Sub_label2) > 1:
                                    del Sub_label2[next(iter(Sub_label2))]  
                                    print(Sub_label2)
                               ref_sub[0] = eval(str(Sub_label)),eval(str(Sub_label2))
                               Pins_list.append(ref_sub.get(0))
                               Pins_label[mcu_code] = Pins_list
                               #print(Pins_list)
                               #Pins_label[mcu_name[0]] = Pins_list
                               #print(Pins_label)
                               Pins_label.get(mcu_code)
                               #for r in Pins_label.get(mcu_code): 
                               #                print(r)
                     return json.dumps(Pins_label)
class Internal_Publish_subscriber(object): 
        
        def Publisher_dict(self,ip,input_message,port):
            try: 
              exec("sock_"+str(port)+" =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
              jsondata = json.dumps(input_message)
              message = pickle.dumps(jsondata)
              exec("sock_"+str(port)+".sendto(message,(ip,port))") # Sending the json data into the udp  
            except ValueError: 
                  print("Connection error via ip: ",str(ip))
                  return 
        def Publisher_string(self,ip,input_message,port):
            try: 
              sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 
              
              message = input_message.encode() # Encode string to byte
              sock.sendto(message,(ip,port)) # Sending the json data into the udp  
            except: 
                  print("Connection error via ip: ",str(ip))  
        def Subscriber_dict(self,ip,buffer_size,port): 
            try: 
               exec("sock_"+str(port)+" =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
               address = (ip,port) 
               exec("sock_"+str(port)+".bind(address)") 
               exec("global data; data,addr"+ "= sock_"+str(port)+".recvfrom("+str(buffer_size)+")") # Btting the bit operating 
               received = pickle.loads(data)
               message = json.loads(received)
               exec("print(message,type(message),addr)")
               return message
            except:
                print("Subscriber connection value error at ip: ",ip,port) # Getting the report on the ip and port value 
        def Subscriber_string(self,ip,buffer_size,port): 
            try: 
               exec("sock_"+str(ip)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)") 
               address = (str(ip),port) 
               exec("sock_"+str(port)+".bind(address)")  
               exec("global data; data,addr"+ "= sock_"+str(port)+".recvfrom("+str(buffer_size)+")") # Btting the bit operating 
               message = data.decode()
               exec("print(message,type(message),addr)") 
               return message   
            except: 
                print("connection error via ip: ",str(ip))

#Sensors and actuator algorithm type of category 
class Action_control(object): 
           #GPIO output serial/local_gpio 
           #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
           # Local GPIO
           def DC_motor_driver_set(self,number_index,direction,gipoL,gpioR,pwm,boolean):  # DC motor driver with on-board gpio 
                  #This function working on 2 pins input from the gpioL gpioR at the same time from the input
                  exec("Motor_"+str(number_index)+" = PhaseEnableMotor("+str(gipoL)+","+str(gpioR),+"pwm="+boolean+","+"pin_factory=None)")  
                  exec("Motor_"+str(number_index)+"."+direction+"(speed = "+str(pwm))          
           def Stepper_motor_driver(self,GPIOA,GPIOB,GPIOC,GPIOD,g_code):  # Unipolar stepper motor control with stepper_motor        
                  pass 
           def BLDC_motor_Driver(self,GPIO,pwm):# BLDC motor driver 
                  pass
           #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
           def Serial_mcu_connect(self,component_name,number,Serialdev): 
                 exec(component_name+"_"+str(number)+" = pyfirmata.ArduinoMega('"+str(Serialdev)+"')") #Choosing the data selection from the sequence conditioning  

           #Serial GPIO 
           def Serial_DC_motordriver(self,mcu_number,number,motor_name,speed,signal_type,serialdev,gpiol,gpior): # Getting the motor name and the speed of the motor 
                  # Convertting this into the execution 
                  # using the pins mapping from the stm32F103C8T6 
              if mcu_number == "STM32F103C8TX": # Getting the microcontroller series 
                  #hardware = pyfirmata.ArduinoMega(serialdev) # Getting the serial dev input here for example /dev/ttyACM0 , /dev/ttyUSB0 
                  exec("motorl_"+str(number) +" = "+str(motor_name)+".get_pin('d:"+str(number[0])+":"+str(signal_type))
                  exec("motorr_"+str(number) +" = "+str(motor_name)+".get_pin('d:"+str(number[0])+":"+str(signal_type))
                  exec("if gpiol != 0:")
                  exec("    motorl_"+str(number)+".write(speed)")
                  exec("    motorr_"+str(number)+".write(0)") 
                  exec("if gpior !=0:") 
                  exec("    motorl_"+str(number)+".write(0)")
                  exec("    motorr_"+str(number)+".write(speed)")

              if mcu_number == "STM32F303K8TX":
                  #hardware = pyfirmata.ArduinoMega(serialdev) # Getting the serial dev input here for example /dev/ttyACM0 , /dev/ttyUSB0 
                  exec("motorl_"+str(number) +" = "+str(motor_name)+".get_pin('d:"+str(number[0])+":"+str(signal_type))
                  exec("motorr_"+str(number) +" = "+str(motor_name)+".get_pin('d:"+str(number[0])+":"+str(signal_type))
                  exec("if gpiol != 0:")
                  exec("    motorl_"+str(number)+".write(speed)")
                  exec("    motorr_"+str(number)+".write(0)") 
                  exec("if gpior !=0:") 
                  exec("    motorl_"+str(number)+".write(0)")
                  exec("    motorr_"+str(number)+".write(speed)")
                  
           def Serial_stepper_driver(self,serialdev,g_code): # Getting the stepper motor board name to classify the board 
                                    
                  pass 
           def Serial_BLDC_motor_Driver(self,GPIO,pwm): 
                   
                  pass
           def Serial_Servo_motor(self,number,motor_type,angle): #Build the servo motor from scratch using the control theory algorithm to calibrate the angle of the servo motor 

                   pass 
           #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

           #I2C GPIO 
           def I2C_DC_motordriver(self,i2c_address,gpiol,gpior):
                  pass 
           def I2C_servo_motor(self,name_servo,angle,pins):
                try:  
                    #pca = PCA9685(i2c)
                    #pca.frequency = 50
                    exec(str(name_servo)+"_servo = "+"servo.Servo(pca.channels["+str(pins)+"]") # Getting the pins number of the servo 
                    exec(str(name_servo)+"_servo.angle = "+str(angle)) # Getting the angle of the servo to activate the function of the motion system
                except:
                     print("PCA9685 I2C connection error please check your device connection ") 
           def I2C_stepper_driver(self,i2c_address,g_code): 

                  pass 

           def I2C_BLDC_motor_Driver(self,GPIO,pwm): 
                  pass  
           #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
class Visual_Cam_optic(object):  # Calling the camera and optic devices input for image processing publish and subscriber on one platform to be easy to movidy with one library 
           #Camera data in the raw input      
           def Camera_raw(self,cam_num,Buffers,portdata,ip_number): # Getting the raw image of the camera
                # Running the full function of the for loop capability to publish the data from this function individually   
                exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                exec("server_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                exec("server_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")                   
                exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)
                exec("print(host_ip_"+str(cam_num)+")")
                exec("port = "+str(portdata)) # Getting the portdata from the list to getting the camera data
                exec("socket_address_"+str(cam_num)+" = (host_ip_"+str(cam_num)+","+str(portdata)+")")
                exec("server_socket_"+str(cam_num)+".bind(socket_address_"+str(cam_num)+")")
                exec("print('Listening at_camnum"+str(cam_num)+":',socket_address_"+str(cam_num)+")") # Getting the raw camera data as a publisher to publish the camera data 
                exec("vid_"+str(cam_num)+" = cv2.VideoCapture("+str(cam_num)+")") #  replace 'rocket.mp4' with 0 for webcam
                exec("fps_"+str(cam_num)+","+"st_"+str(cam_num)+","+"frames_to_count_"+str(cam_num)+","+"cnt_"+str(cam_num)+" = (0,0,20,0)")
                exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tmsg_"+str(cam_num)+","+"client_addr_" +str(cam_num)+" = server_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num) +")"+"\n\t\tprint('GOT connection from',client_addr_"+str(cam_num)+")"+"\n\t\tWIDTH=400"+"\n\t\twhile(vid_"+str(cam_num)+".isOpened()):"+"\n\t\t\t_"+str(cam_num)+",frame_"+str(cam_num)+" = vid_"+str(cam_num)+".read()\n\t\t\tframe_"+str(cam_num)+" = imutils.resize(frame_"+str(cam_num)+",width=WIDTH)\n\t\t\tencoded_"+str(cam_num)+","+"buffer_"+str(cam_num)+" = cv2.imencode('.jpg',frame_"+str(cam_num)+",[cv2.IMWRITE_JPEG_QUALITY,80])\n\t\t\tmessage_"+str(cam_num)+" = base64.b64encode(buffer_"+str(cam_num)+")"+"\n\t\t\tserver_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",client_addr_"+str(cam_num)+")"+"\n\t\t\tframe_"+str(cam_num)+" = cv2.putText(frame_"+str(cam_num)+",'FPS: '+str(fps_"+str(cam_num)+"),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)") # Getting the frame from the video input 
           
           def Camera_subscriber(self,cam_num,Buffers,portdata,ip_host): # Getting the host ip data 
                  #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 

           def Camera_QR(self,cam_num,Buffers,portdata,port_message,ip_number):  # Getting the raw camera image 
                  #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  exec("global x")
                  exec("global y") 
                  # Start the loop of frame rate read
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")"+"\n\t\tif _"+str(cam_num)+":"+"\n\t\t\tImage_"+str(cam_num)+" = cv2.cvtColor(frame_"+str(cam_num)+", cv2.COLOR_BGR2RGB)"+"\n\t\t\ttry:"+"\n\t\t\t\tbarcodes_"+str(cam_num)+" = pyzbar.decode(Image_"+str(cam_num)+")"+"\n\t\t\t\tprint(barcodes_"+str(cam_num)+")"+"\n\t\t\t\tfor barcode_"+str(cam_num)+" in barcodes_"+str(cam_num)+":"+"\n\t\t\t\t\t(x, y, w, h) = barcode_"+str(cam_num)+".rect"+"\n\t\t\t\t\tcv2.rectangle(Image_"+str(cam_num)+", (x, y), (x + w, y + h), (0, 0, 255), 2)"+"\n\t\t\t\t\tbarcodeData_"+str(cam_num)+" = barcode_"+str(cam_num)+".data.decode('utf-8')"+"\n\t\t\t\t\tbarcodeType_"+str(cam_num)+" = barcode_"+str(cam_num)+".type"+"\n\t\t\t\t\ttext_"+str(cam_num)+"= '{} {}'.format(barcodeData_"+str(cam_num)+", barcodeType_"+str(cam_num)+")"+"\n\t\t\t\tprint('Text reading from qr code',text_"+str(cam_num)+")"+"\n\t\t\t\tprint('Coordinate ',text_"+str(cam_num)+",x,y)"+"\n\t\t\t\tglobal message;message_data = {'Message':text_"+str(cam_num)+",'X':x,'Y':y}"+"\n\t\t\t\tprint('from message ',message_data)"+"\n\t\t\t\tQR_"+str(cam_num)+" = Internal_Publish_subscriber()"+"\n\t\t\t\tQR_"+str(cam_num)+".Publisher_dict('"+str(ip_number)+"',message_data,"+str(port_message)+")"+"\n\t\t\texcept:"+"\n\t\t\t\tprint('No QRcode detected!')") # Getting the return of the frame from the loop and 
                  
                  

           def Camera_OCR(self,cam_num,Buffers,portdata,port_message,ip_number):
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 
                  
                  

           def Camera_yolo(self,cam_num,Buffers,portdata,port_message,ip_number): # Getting the raw yolo image 

                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 

           def Camera_Face_recognition(self,cam_num,Buffers,portdata,port_message,ip_number):
 
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 

           def Camera_Visual_to_text(self,cam_num,Buffers,portdata,port_message,ip_number): 
                  exec("BUFF_SIZE_"+str(cam_num)+" = "+str(Buffers))
                  exec("client_socket_"+str(cam_num)+" = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)")
                  exec("client_socket_"+str(cam_num)+".setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE_"+str(cam_num)+")")
                  exec("host_name_"+str(cam_num)+" = socket.gethostname()")
                  exec("host_ip_"+str(cam_num)+" = '"+str(ip_number)+"'") #  socket.gethostbyname(host_name)                  
                  exec("print(host_ip_"+str(cam_num)+")") #Getting the  
                  exec("port_"+str(cam_num)+" = "+str(portdata)) 
                  exec("message_"+str(cam_num) +" = "+"b'Hello'")
                  exec("client_socket_"+str(cam_num)+".sendto(message_"+str(cam_num)+",(host_ip_"+str(cam_num)+","+"port_"+str(cam_num)+"))")
                  exec("fps_"+str(cam_num)+",st_"+str(cam_num)+",frames_to_count_"+str(cam_num)+",cnt_"+str(cam_num)+" = (0,0,20,0)")
                  # Start the loop of frame rate read
                  
                  exec("for r_"+str(cam_num)+" in count(0):"+"\n\t\tpacket_"+str(cam_num)+",_"+str(cam_num)+" = client_socket_"+str(cam_num)+".recvfrom(BUFF_SIZE_"+str(cam_num)+")"+"\n\t\tdata_"+str(cam_num)+" = base64.b64decode(packet_"+str(cam_num)+",' /')"+"\n\t\tnpdata_"+str(cam_num)+" = np.fromstring(data_"+str(cam_num)+",dtype=np.uint8)"+"\n\t\tframe_"+str(cam_num)+" = cv2.imdecode(npdata_"+str(cam_num)+",1)"+"\n\t\tprint(frame_"+str(cam_num)+")") 
                  

           
           #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

#a = Internal_Publish_subscriber() 
#a.Publisher_dict("127.0.0.1",{"input":9048},5080) # Sending the data type dicationary now thinking about getting the value input from json file and convert data into the specific data type 

#Getting the input from the hiroku json data component selector to classify the type of the data in the list and getting the new code to generate 
data_ex = {"actuator_1":3,"microcontroller_1":[5,7],"gpio_1":8} # list json to classify and get the message from each devices connected with it
#Check the sensor and the devices hardware connection
#Getting the data input to the pub with the key and value of dict representing the data in the topic and message data type 

def Create_node_pub(topic,message,addresses,initial_port): # Create the node public from the json file input from component selection and define function of the code in the node to control 
       #Create the module of the node component input into the function                
       #Before running the loop of publisher check that the module node is created
       port = initial_port
       address = addresses
       exec("pub_"+str(port)+ "= Internal_Publish_subscriber()")  
       exec("pub_"+str(port)+".Publisher_dict('"+str(address)+"',{"+"'"+str(topic)+"'"+":"+str(message)+"},"+str(port)+")") # Sending the data type dicationary now thinking about getting the value input from json file and convert data into the specific data type 
       
#Create the sensor receiver as the subscriber for each sensor parameter input from the json component input but this function going to define by json type code generator 
def Create_node_sub(t,addresses,buffer,initial_port):
      
           port = initial_port 
           address = addresses        
           exec("sub_"+str(t)+" = Internal_Publish_subscriber()")  
           exec("global data_return;data_return"+" = sub_"+str(t)+".Subscriber_dict('"+str(address)+"',"+str(buffer)+","+str(port)+")")
           return  data_return 

def Create_i2c_Servo():
        servo_1 = Action_control()
        servo1.I2C_servo_motor('servo_leg',90,1) # define the name angle and pin of the servo to connect into the board 
        pass 

def microcontroller_info_dat(mcu_code_name): 
            exec("mcu_"+str(mcu_code_name)+" = Microcontroller_pins()") 
            exec("global data_mcu; data_mcu" +" = mcu_"+str(mcu_code_name)+".request_mcu('"+str(mcu_code_name)+"')") # Getting the request mcu data 
            return data_mcu 

# Non execution pub node 
def Camera_pub_node(): # running all theses in exec 
       cam1 = Visual_Cam_optic()
       cam1.Camera_raw(0,65536,9800,'192.168.50.43')        

# Non execution sub node 
def Camera_sub_node(): # Input the function into the the command all list of computer vision are { Camera_raw , Camera_QR, Camera_OCR, Camera_yolo, Camera_face_recognition, Camera_Visual_to_text}
  
       cam1i = Visual_Cam_optic() 
       cam1i.Camera_QR(0,65536,9801,5020,'192.168.50.192')
    
     
def mcu1():
   mcu_data = "STM32F103CBTx"
   mcu_list =  microcontroller_info_dat(mcu_data)
   print(mcu_list)
def mcu2(): 
   mcu_data = "STM32F303K8Tx"
   mcu_list =  microcontroller_info_dat(mcu_data)
   print(mcu_list)

# Configure port at the node generator 
# Before requesting this publisher and the subscriber 
message={"speed":5,"Angle":90} # Sending message via exec code create more variable to input instance load of message into the server publisher
def pub1():
    Create_node_pub("actuators_2",[5,7],"127.0.0.1",5080) # Getting the node created from the json component selection 
def pub2():
    Create_node_pub("Servo_1",message,"127.0.0.1",5081)
#Create_node_pub("Servo_2",message,"192.168.50.192",5020) 
#data_out = Create_node_sub(1,"127.0.0.1",4096,5090) # Getting the local message 
#print(data_out)

if __name__=="__main__":
          p1 = threading.Thread(target=pub1)
          p2 = threading.Thread(target=pub2) 
          p3 = threading.Thread(target=mcu1) 
          p4 = threading.Thread(target=Camera_sub_node)
          p5 = threading.Thread(target=Camera_pub_node)
          p1.start() 
          p2.start()
          p3.start()
          p4.start() 
          p5.start()
          p1.join()
          p2.join()
          p3.join() 
          p4.join()
          p5.join()
          
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#Create the code from json input firmware writer 
#data_ref = {"if_1":{"parameter_1":{"in":["input_parameter","list(data_ex)"]},"for_1":['i',"in range(0,10)"],"print_1":['i']}}  

#for r in range(0,len(data_ref)): 
#       print(list(data_ref)[r],list(data_ref)[r][0])
#       for inner_list in range(0,len(list(data_ref.get(list(data_ref)[r])))):
#                 print(list(data_ref.get(list(data_ref)[r]))[inner_list])
      


#b = Internal_Publish_subscriber() 
#data_return = b.Subscriber_dict("127.0.0.1",4096,5090) 
#print(data_return)

#c = Internal_Publish_subscriber() 
#c.Publisher_dict("127.0.0.1",{"input_1":5048},5040)
