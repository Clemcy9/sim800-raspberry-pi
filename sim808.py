#!/usr/bin/python
# -*- coding:utf-8 -*-


# RPI Sim808    ||      Raspberry Pi
#     C_PW      ||          GPIO 27
#      PWK      ||          GPIO 17
#      TxD      ||          RxD (GPIO 15)
#      RxD      ||          TxD (GPIO 14)

# sudo python sim808.py
import pickle
import os
import time
import serial
import RPi.GPIO as GPIO
from time import sleep
import datetime
import subprocess 
from stringexec import parseString, writeString
#Setup gpio pin thuc hien mot so chuc nang dac biet
#C_PWpin = 27        # chan C_PW dieu khien nguon cap cho RPI Sim808 Shield
PWKpin  = 4        # chan PWK : bat/tat RPI Sim808 Shield

#data = ''
#with open('home/pi/log1.txt', 'r') as myfile
#    data = myfile.read()

# setup serial
ser = serial.Serial(
    port = '/dev/seriol0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
)
# ser2 = serial.Serial(
#     port = '/dev/ttyACM0',
#     baudrate = 9600
# )
# setup GPIO
GPIO.setmode(GPIO.BCM)
#GPIO.setup(C_PWpin, GPIO.OUT)
GPIO.setup(PWKpin, GPIO.OUT)

# Path to script folder
binPath = "/home/pi/sim800.Pi/bin/"
defPath = "/home/pi/"

#********************************************************************
# @GSM_Power() khoi dong nguon cho module SIM
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
def GSM_MakeCall():
    print ("dialing...\n")
    ser.write(f'ATD{num}\r\n'.encode('utf-8'))  # Goi dien toi sdt 012345678
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
    print ("\n\nBat dau test module Sim808 voi Raspberry Pi ... \n")
    print ("Bat nguon cho module Sim808...\n")
    GSM_Power()         # Bat nguon cho module
    GSM_Init()      # Khoi dong module
    #GSM_MakeCall()         # Tao cuoc goi
    #GSM_MakeSMS()      # Tao tin nhan
    print ("done")
    dataserial = ''
    voltage = ''
    while (1):
        dataserial= dataserial + ser.readline()
#	for i in range(0,1):
 #           ser2.write('D')
  #  	    voltage = ser2.readline()
   # 	    writeString('outputvoltage.txt', voltage)
    #	    print (voltage)
############ Add more function here ##########
        if(len(dataserial)>0):
            ###### Help ######
            print(dataserial)
            # if (dataserial.find("NORMAL POWER DOW") > 0):
            # 	GSM_Power()
            # 	GSM_Init()
            # 	dataserial = ''
            # if(dataserial.find("daily")>0):
            #     dailyInfo()
            #     ser2.write('D')
            #     voltage = ser2.readline()
            #     writeString('/home/pi/sim800/outputvoltage.txt', voltage)
            #     myfile=open('/home/pi/sim800/dailyLog.txt','r')  #### path to your daily log file 
            #     dataDaily=myfile.read()
            #     GSM_MakeSMS(dataDaily)
            #     dataserial=''

            # if(dataserial.find("help")>0):
            #     print (dataserial)
            #     datalog = ''
            #     myfile=open('/home/pi/sim800/help/help.txt', 'r') ## path to your help file 
            #     datalog = myfile.read()
            #     GSM_MakeSMS(datalog)
            #     dataserial=''

            # ##### track your file system ###
            # if(dataserial.find("log")>0):
            #     print( dataserial)
            #     datalog = ''
            #     killIno()
            #     ## read data from log file
            #     myfile= open('/home/pi/sim800/inotiwait.txt', 'r') #### Path to your log file 
            #     datalog = myfile.read()
            #     GSM_MakeSMS(datalog)
            #     dataserial=''
            #     #run inotiwait again
            #     Inowait()
            # ########## change DPI of your scanner
            # if(dataserial.find("dpi")>0):
            #     writeString('/home/pi/sim800/outputDPI.txt', parseString(dataserial, 'dpi '))
            #    	dataserial=''
            #    	changeDPI()
               
            # #turn off sim module########s
            # if(dataserial.find("turn off sim")>0):
            #     #print dataserial
            #     GSM_Power()
            #     ser.close()
            #     GPIO.cleanup()
            #     dataserial=''
            # #### turn off your Pi ####
            # if(dataserial.find("turn off")>0):
            #     #print dataserial
            #     GSM_Power()
            #     ser.close()
            #     GPIO.cleanup()
            #     dataserial=''
            #     os.system("sudo shutdow now")
            # #### change time of your cronjob 
            # if(dataserial.find("time")>0):
            # 	writeString('/home/pi/sim800/outputTime.txt', parseString(dataserial, 'time '))
            #   	dataserial=''
            #    	changeTime()
            # #daily infomation########
            # timecheck=datetime.datetime.now()
            # if timecheck.hour==10 and timecheck.minute==12 and timecheck.second<5: #### change time that you want to receive your sms
            #     dailyInfo()
            #     myfile=open('/home/pi/sim800/dailyLog.txt','r')  #### path to your daily log file 
            #     dataDaily=myfile.read()
            #     print(dataDaily)
            #     GSM_MakeSMS(dataDaily)

        else :
        	print('waiting...\r\n')
        time.sleep(0.5)
except KeyboardInterrupt:
    ser.close()
finally:
    #print "End!\n"
    ser.close()
    GPIO.cleanup()      # cn up all port
