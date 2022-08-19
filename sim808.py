#!/usr/bin/python
# -*- coding:utf-8 -*-


# RPI Sim808    ||      Raspberry Pi
#     C_PW      ||          GPIO 27
#      PWK      ||          GPIO 17
#      TxD      ||          RxD (GPIO 15)
#      RxD      ||          TxD (GPIO 14)

# sudo python sim808.py
import time
import serial
import RPi.GPIO as GPIO
from time import sleep
import datetime 
#Setup gpio pin thuc hien mot so chuc nang dac biet
#C_PWpin = 27        # chan C_PW dieu khien nguon cap cho RPI Sim808 Shield
PWKpin  = 4        # chan PWK : bat/tat RPI Sim808 Shield

#data = ''
#with open('home/pi/log1.txt', 'r') as myfile
#    data = myfile.read()

# setup serial
ser = serial.Serial(
    port = '/dev/serial0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
)
# setup GPIO
GPIO.setmode(GPIO.BCM)
#GPIO.setup(C_PWpin, GPIO.OUT)
GPIO.setup(PWKpin, GPIO.OUT)

# Path to script folder
binPath = "/home/pi/sim800.Pi/bin/"
defPath = "/home/pi/"

#********************************************************************
# @GSM_Power() 
#********************************************************************
def GSM_Power():
    print ("Bat nguon cho module Sim808...\n")
    GPIO.output(PWKpin, 1)
    time.sleep(2)
    GPIO.output(PWKpin, 0)
    time.sleep(10)
    return


#********************************************************************
# @GSM_Init() 
#********************************************************************
def GSM_Init():
    ser.write(b'ATE0\r\n')              # turn off (Echo mode)
    time.sleep(2)
    ser.write(b'AT+IPR=9600\r\n')       # Dat toc do truyen nhan du lieu 9600bps
    time.sleep(2)
    ser.write(b'AT+CMGF=1\r\n')         # Chon che do text mode
    time.sleep(2)
    ser.write(b'AT+CLIP=1\r\n')         # caller identification 1 on, 0 off
    time.sleep(2)
    ser.write(b'AT+CNMI=2,2\r\n')       # how to process new recieved message
    time.sleep(2)
    return

#********************************************************************
# @GSM_MakeCall() 
#********************************************************************
def GSM_MakeCall(num):
    print ("dialing...\n")
    ser.write(f'ATD{num}\r\n'.encode('utf-8'))  # calling num
    time.sleep(2)
    ser.write(b'ATH\r\n') #put the on answering mode
    time.sleep(2)


#********************************************************************
# @GSM_MakeSMS() tao cuoc goi
#********************************************************************
def GSM_MakeSMS(num,data):
    print ("sending...\n")
    ser.write(f'AT+CMGS=\"{num}\"\r\n'.encode('utf-8'))    # change your phone number here
    time.sleep(5)
    ser.write(f'{data}'.encode('utf-8'))
    ser.write(b'\x1A')      # Gui Ctrl Z hay 26, 0x1A de ket thuc noi dung tin nhan va gui di
    time.sleep(5)


# Simple example :
try:
    print ("\n\ntesting the module Raspberry Pi ... \n")
    GSM_Power()          
    GSM_Init()      
    #GSM_MakeCall()         
    #GSM_MakeSMS()      
    print ("done")
    dataserial = ''
    voltage = ''
    while (1):
        dataserial= dataserial + ser.readline()
        ############ Add more function here ##########
        if(len(dataserial)>0):
            ###### Help ######
            print(dataserial)
        else :
            print('waiting...\r\n')
        time.sleep(0.5)

except KeyboardInterrupt:
    ser.close()
finally:
    #print "End!\n"
    ser.close()
    GPIO.cleanup()      # cn up all port
