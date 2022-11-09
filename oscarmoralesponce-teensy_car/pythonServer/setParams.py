import serial
import serial.tools.list_ports
from struct import *
import array
import sys
from sys import stdout


#C  = 67
#O  = 81
#size = 32
#Data
#    address H  0 
#    length  B  1
#     bytes  B  3
#checksum      4 

def payloadPID(address, proportional, integral, derivate) :
  ch = 0
  pay = pack('>BHIII', address, 12, proportional,integral, derivate) 
  a = array.array('b', pay).tolist()

  for i in range(0, len(a)):
    ch = (ch + a[i]) % 256
  for i in range(len(a), 32):
     pay = pay + pack('B', 0)

  pay = pack('BBB', 67, 79, 32) + pay + pack('B', ch)

  return pay

def payloadOffset(lateraloffset, off) :
  ch = 0
  pay = pack('BHbbB', 52, 3, lateraloffset, off, 0) 
  print("pay", len(pay))
  a = array.array('b', pay).tolist()

  for i in range(0, len(a)):
    ch = (ch + a[i]) % 256
  for i in range(len(a), 32):
     pay = pay + pack('B', 0)

  pay = pack('BBB', 67, 79, 32) + pay + pack('B', ch)
  print(pay)

  return pay

def payload(idData) :
  ch = 0
  pay = pack('HBB', 0, 1, idData)
  a = array.array('b', pay).tolist()
  for i in range(0, len(a)):
    ch = (ch + a[i]) % 256
  for i in range(len(a), 32):
     pay = pay + pack('B', 0)
  
  pay = pack('BBB', 67, 79, 32) + pay + pack('B', ch)
  return pay



if len(sys.argv) <= 1:
  print("Usage setParamaters.py [-id num] [-offset num1 num2] [-pid type proportional integral derivate]")
  exit(0)
else :
  # configure the serial connections (the parameters differs on the device you are connecting to)
  #comlist = serial.tools.list_ports.comports()
  for i in range(0, 2):
      try:  
        print("Trying openning port /dev/ttyACM{0}".format(i))
        #if element[1] == 'USB Serial':
        ser = serial.Serial(port='/dev/ttyACM{0}'.format(i), baudrate=115200)
        print("Connected")
        if sys.argv[1] == "-id":
            print("writing id", sys.argv[2])
            ser.write(payload(int(sys.argv[2])))
        if sys.argv[1] == "-offset":
            print("writing offset", int(sys.argv[2]), int(sys.argv[3]))
            ser.write(payloadOffset(int(sys.argv[2]), int(sys.argv[3])))
                      
        if sys.argv[1] == "-pid" and len(sys.argv) >= 5:
            if sys.argv[2] == "speed":
                print("writing pid speed")
                ser.write(payloadPID(4, int(float(sys.argv[3])*1000), int(float(sys.argv[4])*1000), int(float(sys.argv[5])*1000)))
                
            if sys.argv[2] == "lspeed":
                print("writing pid lateral speed")
                ser.write(payloadPID(16, int(float(sys.argv[3])*1000), int(float(sys.argv[4])*1000), int(float(sys.argv[5])*1000)))
            if sys.argv[2] == "rotation":
                print("writing pid rotation")
                ser.write(payloadPID(28, int(float(sys.argv[3])*1000), int(float(sys.argv[4])*1000), int(float(sys.argv[5])*1000)))
            
            if sys.argv[2] == "correction":
                print("writing pid correction")
                ser.write(payloadPID(40, int(float(sys.argv[3])*1000), int(float(sys.argv[4])*1000), int(float(sys.argv[5])*1000)))
 
        break
      except Exception, e:
         print(e)
         pass
  
  


