import serial
from struct import *
import array
import sys


#C  = 67
#O  = 81
#size = 32
#Data
#    address H  offset + num (29)  
#    length  B  29
#     bytes  B  data
#checksum      4 


def payload(offset, fo) :
  ch = 0
  pay = pack('HB', offset, 29)
  
  c = fo.read(29)
  a = array.array('b', c)
  a = a.tobytes()
  
 
  for c in a:
     pay = pay + pack('B', c)
     #ch = (ch + c) % 256
  for i in range(0, 29 - len(a)):
     pay = pay + pack('B', 0)
     #ch = ch + 0   
     
  a = array.array('b', pay)
  a = a.tobytes()
  
  for i in range(0, 32):
    ch = (ch + a[i]) % 256
  pay = pay + pack('B', ch)
  return pay 



if len(sys.argv) <= 2:
  print("Usage sendByteCode.py offset filename")
  exit(0)
else :
  # configure the serial connections (the parameters differs on the device you are connecting to)
  ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
  ser.isOpen()
  
  fo = open(sys.argv[2], "rb")
  offset = int(sys.argv[1])
  num = 0
  fo.seek(0, 2)
  lenght = fo.tell()
  fo.seek(0,0)
  #print(fo.tell(), lenght)
  while fo.tell() < lenght:  
      pay = payload(offset + 29*num, fo)
      num = num + 1
      #print(pay)
      
      ser.write(pack('BBB', 67, 79, 32))
      ser.write(pay)
      
      if fo.tell() > 2000:
        break
      
  fo.close()
  ser.close()

  
  
  
  


