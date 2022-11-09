import serial
from struct import *
import array
import sys
from sys import stdout
from threading import Timer, Thread, Semaphore
from time import sleep
import queue
import serial.tools.list_ports
import time
from LocalDynamicMap import LocalDynamicMap
from statemachinecom import StateMachinCommunicaton 
from virtualmachine import VirtualMachine
from autonomouscar import AutonomousCar

UART_FIRST_BYTE = 15
UART_SECOND_BYTE = 201

ser = None
sem = Semaphore()
myQueue = queue.Queue(1)
commandQueue = queue.Queue(1)
localDynamicMap = LocalDynamicMap()


def payload(maxTime, speed, orientation, distance, direction, stop) :
  ch = 0
  length = 20
  #print(maxTime, speed, orientation, distance, direction, stop)
  pay = pack('hhhhBBBBII', maxTime, int(speed), int(orientation), int(distance), direction, stop, 0,0,0, 0)
  a = array.array('b', pay)
  
  for i in range(0, length):
    ch = (ch + a[i]) % 256
    
  pay = pack('BBB', UART_FIRST_BYTE, UART_SECOND_BYTE, length) + pay + pack('B', ch)
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

def readCommand():
   global code
   while True:
     command = commandQueue.get()
     # maxTime, speed, orientation, distance, direction, stop
     # print(command.speed)
     pay = payload(command.maxTime, command.speed, command.orientation, command.distance, \
         command.direction, command.stop)
     if ser is not None:
      try:
        sem.acquire()
        #print("commnad", command.speed) 
        ser.write(pay)
        sem.release()
      except:
        sem.release()
        pass  
        
     
    
def  connect():
  global ser
  comlist = serial.tools.list_ports.comports()
  for p in comlist:
    try:
      print(p[1])
      if 'ttyACM' in p[0] or ('COM' in p[0] and 'CH340' not in p[1]):  
        ser = serial.Serial(port=p[0], baudrate=115200)
        print(f"Connected  to {p[0]}")
        break
    except Exception as e:
      print(e)

def readSerial():
  global ser
  while 1: 
      try:
        sem.acquire()
        cont = ser.read_all()
        sem.release()
        for c in cont:
            myQueue.put(c)
        time.sleep(0.05)
      except Exception as e:
        sem.release()
        time.sleep(0.1)
        ser = None
        connect()
     

def testMemory() :
  global ser
  while 1: 
      try:
        
        sem.acquire()
        msg = payloadOneByte(2, 2)
        if ser is not None:
          print("Sending Memory")
          ser.write(msg)
        sem.release()
        time.sleep(2)
      except:
        sem.release()
        pass
        


autonomousCarAlgo = AutonomousCar()
stateMachineCom = StateMachinCommunicaton(myQueue, localDynamicMap)
virtualMachine = VirtualMachine(0.1, localDynamicMap, autonomousCarAlgo, commandQueue)
thread = Thread(target = readSerial)
thread2 = Thread(target = readCommand)
#thread3 = Thread(target = testMemory)
thread.start()
thread2.start()
#thread3.start()
stateMachineCom.start()
virtualMachine.start()
thread.join()
thread2.join()
stateMachineCom.join()  
stateMachineCom.start()

  
  
  


