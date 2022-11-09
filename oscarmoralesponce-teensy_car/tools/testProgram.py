import serial
from struct import *
import array
import sys
from sys import stdout
from threading import Timer, Thread, Semaphore
from time import sleep

sem = Semaphore()




code = 5
distance = 100 
offset = 20 
sensor1 = 0
sensor2 = 0
sensor3 = 0

#C  = 67
#O  = 81
#size = 12
#Data
#    uint16_t code;
#    int16_t offset_x, offset_y;
#    int16_t sensor1_dist; 
#    int16_t sensor2_dist; 
#    int16_t sensor3_dist;
#checksum      




def payload(code, distance, offset, sensor1, sensor2, sensor3) :
  ch = 0
  length = 12
  pay = pack('hHHHHH', code, distance, offset, sensor1, sensor2, sensor3)
  a = array.array('b', pay)
  a = a.tobytes()
  
  for i in range(0, 12):
    ch = (ch + a[i]) % 256
    
  pay = pack('BBB', 67, 79, 12) + pay + pack('B', ch)
  return pay


ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
ser.isOpen()

def readSerial():
  while 1: 
     sleep(0.1)
     sem.acquire()
     lines = ser.read_all().splitlines()	
     sem.release()
     for c in lines:
       if len(c) == 6:
         (m1, m2, l, qr, command, ch) = unpack('BBBBBB', c) 
         if m1== 67 and m2 == 79 and (qr + command) == ch:
             print("QR", qr,  "command", command)
       else:
         print(c.decode("utf-8"))
  
def Sensors():
    pay = payload(code, distance, offset, sensor1, sensor2, sensor3)
    sem.acquire() 
    ser.write(pay)  
    sem.release()
    #print("payload ", pay)
    t = Timer( 0.5, Sensors )
    t.start()

thread = Thread(target = readSerial)
thread.start()
Sensors()
thread.join()

  
  
  


