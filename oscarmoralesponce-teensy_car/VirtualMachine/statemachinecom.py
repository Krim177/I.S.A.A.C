from struct import *
import threading
import queue
from LocalDynamicMap import LocalDynamicMap, LocalMap 



UartWaitForFirstStart = 0
UartWaitForSecondStart = 1
UartWaitForLenght = 2
UartWaitForData = 3
UartWaitForCheckSum = 4

UART_FIRST_BYTE = 15
UART_SECOND_BYTE = 201
SIZEOFMAP = 36

class PrintSerial(threading.Thread):
  def __init__(self, strQueue):
    threading.Thread.__init__(self)
    self.strQueue = strQueue

  def run(self):
    while True:
      text = self.strQueue.get()
      print(str(text))


class StateMachinCommunicaton(threading.Thread):
    def __init__(self, data, localDynamicMap) :
      threading.Thread.__init__(self)
      self.strQueue = queue.Queue()
      self.printSerial = PrintSerial(self.strQueue)
      self.printSerial.start()
      self.data = data
      self.localDynamicMap = localDynamicMap

    def run(self) :
      rxUartState = UartWaitForFirstStart
      buffer = []
      stringlist = []

      while True:
        c = self.data.get(True)
        
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
                print(s)
                self.strQueue.put(s)
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
              localMap = LocalMap(posx, posy, speed, mem, memInPro, memInSync, heading, id, \
                    leaderId, OM, tioState, instruction, rssi, prop, Xdistance, \
                    YDistance, param1, param2, globalState, boxWidth,boxlenght, t)
              
              
              self.localDynamicMap.update(localMap)
              #print(posx, posy, id, speed, heading)
            buffer.clear()
            rxUartState = UartWaitForFirstStart
        
    
  
  
  


