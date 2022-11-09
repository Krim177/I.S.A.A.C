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



UartWaitForFirstStart = 0
UartWaitForSecondStart = 1
UartWaitForLenght = 2
UartWaitForData = 3
UartWaitForCheckSum = 4

UART_FIRST_BYTE = 15
UART_SECOND_BYTE = 201
SIZEOFMAP = 36


ser = None
sem = Semaphore()
myQueue = queue.Queue()
strQueue = queue.Queue()


code = 0
distance = 100 
offset = 20 
sensor1 = 0
sensor2 = 0
sensor3 = 0


class LocalDynamicMap:
    def __init__(self, posx, posy, speed, mem, memInPro, memInSync, heading, id, \
              leaderId, OM, tioState, instruction, rssi, prop, Xdistance, \
                YDistance, param1, param2, globalState, boxWidth,boxlenght, t) :
      self.posx = posx
      self.posy = posy
      self.speed = speed
      self.mem = mem
      self.memInPro = memInPro
      self.memInSync = memInSync
      self.heading = heading
      self.id = id
      self.leaderId = leaderId
      self.OM = OM
      self.tioState = tioState
      self.instruction = instruction
      self.rssi = rssi
      self.prop = prop
      self.Xdistance = Xdistance
      self.YDistance = YDistance
      self.param1 = param1
      self.param2 = param2
      self.globalState = globalState
      self.boxWidth = boxWidth
      self.boxlenght = boxlenght
      self.t = t
      


def payload(maxTime, speed, orientation, distance, direction, stop) :
  ch = 0
  length = 20
  pay = pack('hhhhBBBBII', maxTime, speed, orientation, distance, direction, stop, 0,0,0, 0)
  a = array.array('b', pay)
  
  print(len(a))
  for i in range(0, length):
    ch = (ch + a[i]) % 256
    
  pay = pack('BBB', UART_FIRST_BYTE, UART_SECOND_BYTE, length) + pay + pack('B', ch)
  return pay


def printSerial():
  while True:
    text = strQueue.get()
    print(str(text))


def readCommand():
   global code
   while 1:
     # maxTime, speed, orientation, distance, direction, stop
     code =input()
     print(code)
     pay = payload(5000, 20, 0, 20, int(code), 1)
     sem.acquire() 
     ser.write(pay)  
     sem.release()
     #print("payload ", pay)
     #t = Timer( 2.0, SendCommand)
     #t.start()


def stateMachine() :
    rxUartState = UartWaitForFirstStart
    buffer = []
    stringlist = []

    while True:
      c = myQueue.get(True)
      
      if rxUartState == UartWaitForFirstStart:
        if c == UART_FIRST_BYTE:
          rxUartState = UartWaitForSecondStart
        else:
          c1 = chr(c)
          if c1.isprintable():
            stringlist.append(c1)
          else:
            s = ''.join(stringlist)
            if len(s) > 1:
              strQueue.put(s)
              stringlist.clear()
        
      elif rxUartState == UartWaitForSecondStart:
        if c == UART_SECOND_BYTE:
          rxUartState = UartWaitForLenght        
        else:
          rxUartState = UartWaitForFirstStart

      elif rxUartState == UartWaitForLenght:
        # check if the lenght is of the size of the LDMAP, otherwise reject
        # print("lenght ", c)
        if c == SIZEOFMAP:
          ch = 0
          lenght = c
          iByte = 0
          rxUartState = UartWaitForData
        else:
          rxUartState = UartWaitForFirstStart       
      elif rxUartState == UartWaitForData:
          if iByte < lenght:
              ch = (ch + c)%256
              buffer.append(c)
              iByte += 1 
          if iByte >= lenght:
              rxUartState = UartWaitForCheckSum;
      elif rxUartState == UartWaitForCheckSum:
          if ch == c:
            #print(bytes(buffer))
            (posx, posy, speed, mem, memInPro, memInSync, heading, id, \
              leaderId, OM, tioState, instruction, rssi, prop, Xdistance, \
                YDistance, param1, param2, globalState, boxWidth,boxlenght, t) = \
                  unpack('ffhHHHhBBBBBBBhhBBBBBB', bytes(buffer)) 
            print(posx, posy, id, speed, heading)
            
          buffer.clear()
          rxUartState = UartWaitForFirstStart
        
    
def  connect():
  global ser
  comlist = serial.tools.list_ports.comports()
  for p in comlist:
    try:
      if 'ttyACM' in p[0] or 'COM' in p[0] :  
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
     


thread = Thread(target = readSerial)
thread1 = Thread(target= stateMachine)
thread2 = Thread(target = readCommand)
thread3 = Thread(target = printSerial)
thread.start()
thread1.start()
thread2.start()
thread3.start()
thread.join()
thread1.join()
thread2.join()
thread3.join()

  
  
  


