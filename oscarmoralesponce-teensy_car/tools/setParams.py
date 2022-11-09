import serial
import serial.tools.list_ports
from struct import *
import array
import sys
from sys import stdout



UART_FIRST_BYTE = 15
UART_SECOND_BYTE = 201

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

  pay = pack('BBB', UART_FIRST_BYTE, UART_SECOND_BYTE, 32) + pay + pack('B', ch)

  return pay

def payloadOffset(lateraloffset, off) :
  ch = 0
  pay = pack('BHbbB', 40, 3, lateraloffset, off, 0) 
  print("pay", len(pay))
  a = array.array('b', pay).tolist()

  for i in range(0, len(a)):
    ch = (ch + a[i]) % 256
  for i in range(len(a), 32):
     pay = pay + pack('B', 0)

  pay = pack('BBB', UART_FIRST_BYTE, UART_SECOND_BYTE, 32) + pay + pack('B', ch)
  print(pay)

  return pay

def payloadOneByte(address, idData) :
  ch = 0
  pay = pack('HBB', address, 1, idData)
  a = array.array('b', pay).tolist()
  for i in range(0, len(a)):
    ch = (ch + a[i]) % 256
  for i in range(len(a), 32):
     pay = pay + pack('B', 0)
  
  pay = pack('BBB', UART_FIRST_BYTE, UART_SECOND_BYTE, 32) + pay + pack('B', ch)
  return pay



if len(sys.argv) <= 1:
  print("Usage setParamaters.py [-id num] [-offset num1 num2] [-pid type proportional integral derivate]")
  exit(0)
else :
  # configure the serial connections (the parameters differs on the device you are connecting to)
  comlist = serial.tools.list_ports.comports()
  for p in comlist:
    if 'ttyACM' in p[0] or 'COM' in p[0] :  
        ser = serial.Serial(port=p[0], baudrate=115200)
        if sys.argv[1] == "-id":
            ser.write(payloadOneByte(0, int(sys.argv[2])))
        if sys.argv[1] == "-type":
            ser.write(payloadOneByte(1, int(sys.argv[2])))
        if sys.argv[1] == "-offset":
            print(int(sys.argv[2]), int(sys.argv[3]))
            ser.write(payloadOffset(int(sys.argv[2]), int(sys.argv[3])))
                      
        if sys.argv[1] == "-pid" and len(sys.argv) >= 5:
            if sys.argv[2] == "speed":
                ser.write(payloadPID(4, int(float(sys.argv[3])*1000), int(float(sys.argv[4])*1000), int(float(sys.argv[5])*1000)))
                
            if sys.argv[2] == "rotation":
                ser.write(payloadPID(16, int(float(sys.argv[3])*1000), int(float(sys.argv[4])*1000), int(float(sys.argv[5])*1000)))
            
            if sys.argv[2] == "correction":
                ser.write(payloadPID(28, int(float(sys.argv[3])*1000), int(float(sys.argv[4])*1000), int(float(sys.argv[5])*1000)))
 
  
  
  


