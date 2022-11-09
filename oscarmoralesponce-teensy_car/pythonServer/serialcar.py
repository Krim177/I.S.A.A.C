import threading
import time
import serial
from struct import *
import array
import sys
import signal
import logging
import calendar
import subprocess
import os
import serial.tools.list_ports
import queue
#import autopilot

SIZEOFMAP = 34


from shareobjects import ShareObjects 
from serdistance import DistanceSensor
from codereader import CodeReader
from listener import Listener

LINES = 30

serialLines = queue.Queue(50) 


class ShareSerial():
  def __init__(self):
    self.serial = None
    self.isOpen = False
    self.porti = 0
    self.debugPrint = []
    for i in range (0,LINES):
      self.debugPrint.append('')

  def connect(self) :   
      comlist = serial.tools.list_ports.comports()
      for p in comlist:
        try:
          if 'ttyACM' in p[self.porti] or 'COM' in p[self.porti] :  
            self.serial = serial.Serial(port=p[0], baudrate=115200)
            print(f"Connected  to {p[0]}")
            self.isOpen = True
            logging.warning('**** Exeuting Program ***')
        except Exception as e:
          self.porti = (self.porti + 1) % 3
          #print(e)
        
class ReadCommand(threading.Thread):
    def __init__(self, shareObjects):
        threading.Thread.__init__(self)
        self.shareObjects = shareObjects
    def run(self) :
        while True:
            c = sys.stdin.readline().splitlines() 
            if c[0] == 'k':
                exit(0)
                os.system("killall python")
            if c[0].isdigit():
                 self.shareObjects.lock.acquire()
                 self.shareObjects.code = int(c[0])
                 self.shareObjects.lock.release()

class DisplaySerial(threading.Thread):             
  def __init__(self, rate, shareSerial):
    threading.Thread.__init__(self)
    self.shareSerial = shareSerial
    self.rate = rate
    self.debugPrint = shareSerial.debugPrint
    self.printLine = 0
    
  def carCommands(self, code):
    if code == 0:
        return "Reset"
    if code == 1:
        return "Standby"
    if code == 2:
        return "Brake"
    if code == 3:
        return "Forward" 
    if code == 4:
        return "Backward"
    if code == 5:
        return "Left"
    if code == 6:
        return "Right"
    if code == 7:
        return "Rotate"
    
  def printDebug(self) :
    print(chr(27)+'[2j')
    print('\033c')
    print('\x1bc')
    for i in range(0, LINES):
        if i >= 2 and i <= 7:
           print("\033[{0};0H\033[1;31m{1}".format(i+1, self.debugPrint[i]))
        else:
           print("\033[{0};0H\033[0;0m{1}".format(i+1, self.debugPrint[i]))
    self.debugPrint[11] = ""
    
  def run(self) :
    j = 0
    self.end = time.time()
    self.start = time.time()
    
    while True:
        global serialLines    
        c = serialLines.get()
        try:
            debugData = c.decode("ascii", 'replace').split(",")
            #print(debugData)
            if len(debugData) > 0 and debugData[0] == u'DD0':

              self.debugPrint[0] = "Offset:{0},{1} PID(H) ({2},{3},{4}) PID(V) ({5},{6},{7})".format(debugData[1], debugData[2], debugData[3], debugData[4], debugData[5], debugData[6], debugData[7], debugData[8])
              logging.warning(self.debugPrint[0])

            elif len(debugData) > 0 and debugData[0] == u'DD0a':

              self.debugPrint[1] = "PID rotation ({0},{1},{2}) PID correction ({3},{4},{5})".format(debugData[1], debugData[2], debugData[3], debugData[4], debugData[5], debugData[6])
              #print(self.debugPrint[0])
              logging.warning(self.debugPrint[1])
            elif len(debugData) > 0 and debugData[0] == u'DD1':
              self.debugPrint[2] = "Id %s Global State %s Local State %s Second State %s"%(debugData[1], debugData[2], debugData[3], debugData[4])
              self.debugPrint[3] = "Rank %s Active %s Code %s"%(debugData[5], debugData[6], debugData[7])
              logging.warning(self.debugPrint[2])
              logging.warning(self.debugPrint[3])  

            elif len(debugData) > 0 and debugData[0] == u'DD2':
              self.debugPrint[4] = "X: %s Y: %s Speed: %s Heading: %s"%(debugData[1], debugData[2], debugData[3], debugData[4])
              self.debugPrint[5] = "Length:%s Width: %s XDistance: %s YDistance: %s"%(debugData[5], debugData[6], debugData[7], debugData[8])      
              self.debugPrint[6] = "Move:%s Face: %s Interdistance:%s Failures: %s"%(debugData[9], debugData[10], debugData[11], debugData[12]) 
              logging.warning(self.debugPrint[4])
              logging.warning(self.debugPrint[5])
              logging.warning(self.debugPrint[6])

            elif len(debugData) > 0 and debugData[0] == u'DD3':
              self.debugPrint[7] = "Distance %s Offset: %s Angle: %s Sensors dist 1: %s, %s, %s, %s"%(debugData[1], debugData[2], debugData[3], debugData[4],  debugData[5], debugData[6], debugData[7])
              logging.warning(self.debugPrint[7])

            elif len(debugData) > 0 and debugData[0] == u'DD4':
              self.debugPrint[9] = "Command %s Dist: %s Angle: %s yaw %s"%(self.carCommands(int(debugData[1])), debugData[2], debugData[3], (float(debugData[4])*180/3.1415)) 
              logging.warning(self.debugPrint[9])
              #print(self.debugPrint[5])
            elif len(debugData) > 0 and debugData[0] == u'DD5':
              self.debugPrint[10] = "%s Dist error: %s Angle Error: %s"%(debugData[1], debugData[2], debugData[3])
              logging.warning(self.debugPrint[10])
            elif len(debugData) > 0 and debugData[0] == u'DD6':
              ts = calendar.timegm(time.gmtime())
              self.debugPrint[10 + int(debugData[1])] = "%s ID %s Operation mode %s tioState %s State %s"%(ts, debugData[1], debugData[2], debugData[3], debugData[4])
              logging.warning(self.debugPrint[10+ int(debugData[1])])
            
            elif (c[0] != 67 or c[1] != 79):
                ts = calendar.timegm(time.gmtime())
                self.debugPrint[18 + self.printLine] = "Teensy:{0},  {1}".format(ts, c.decode("utf-8", "replace"))
                self.debugPrint[18 + self.printLine] = "Teensy:{0},  {1}".format(ts, c)
                logging.warning(self.debugPrint[16 + self.printLine])
                self.printLine = (self.printLine + 1) % (LINES - 18)
                
        except Exception as e:
          print("Exception", e)

          self.debugPrint[11] = (e, c)
      
        self.end = time.time()
        if self.end - self.start > 0.1:
           self.start = self.end
           self.printDebug()
        
class SerialWriter(threading.Thread):
  def __init__(self, rate, shareObjects, shareSerial):
    threading.Thread.__init__(self)
    self.shareSerial = shareSerial
    self.rate = rate
    self.shareObjects = shareObjects
    self.startTime = 0
    self.endTime = 0
    self.debugPrint = shareSerial.debugPrint

  def boundUnsigned(self, x) :
      if x > 10000: 
          x = 10000
      if x < 0:
          x = 0
      return x
  
  def boundSigned(self, x) :
      if x > 10000: 
          x = 10000
      if x < -10000:
          x =-10000
      return x


  def payload(self, code, distance, offset, angle, sensor0, sensor1, sensor2, sensor3, sensor4):
    ch = 0
    length = 18
    #print(sensor0, sensor1, sensor2, sensor3)
    pay = pack('hHhhHHHHH', self.boundSigned(code), self.boundUnsigned(int(distance)), self.boundSigned(int(offset)), 
            self.boundSigned(int(angle)), self.boundUnsigned(int(sensor0)), self.boundUnsigned(int(sensor1)), 
            self.boundUnsigned(int(sensor2)), self.boundUnsigned(int(sensor3)), self.boundUnsigned(int(sensor4)))
    a = array.array('b', pay).tolist()
    for i in range(0, length):
      ch = (ch + int(a[i])) % 256

    pay = pack('BBB', 67, 79, length) + pay + pack('B', ch)
    return pay

class SerialWriterCommand(SerialWriter):
  def __init__(self, rate, shareObjects, shareSerial, queue):
    super().__init__(self, shareObjects, shareSerial)
    self.queue = queue

  def payload(self, maxTime, speed, orientation, distance, direction, stop):
    ch = 0
    length = 20
    pay = pack('hhhhBBBBII', self.boundSigned(maxTime), self.boundSigned(speed), 
              self.boundSigned(orientation), self.boundSigned(distance), 
              direction, stop, 0,0,0, 0)
    a = array.array('b', pay)
    
    for i in range(0, length):
      ch = (ch + a[i]) % 256
      
    pay = pack('BBB', 67, 80, length) + pay + pack('B', ch)
    return pay
  

  def writeData(self, c):  
    pay = self.payload(code, distance, offset, angle, sensorDist[0], sensorDist[1], sensorDist[2], sensorDist[3], 0)
    try:
      self.shareSerial.serial.write(pay)  # write payload to serial port     
    
    except Exception as e:
      self.shareSerial.isOpen = False
      print(e)
      pass

    self.debugPrint[8] = "New Command:{0} Offset:{1} Angle:{2} Sensors dist: {3}, {4}, {5}, {6}".format(distance, offset, angle, sensorDist[0], sensorDist[1], sensorDist[2], sensorDist[3])
    logging.warning(self.debugPrint[8])

  def run(self) :
    while True:
      c = self.queue.get()
      self.queue.task_done()
      self.writeData(c)

class SerialCarWriter(SerialWriter):
  def __init__(self, rate, shareObjects, shareSerial):
    super().__init__(rate, shareObjects, shareSerial)
  
  
  def writeData(self):  
    self.shareObjects.lock.acquire(True) 
    code = self.shareObjects.code
    distance = self.shareObjects.distance
    offset = self.shareObjects.offset
    angle = self.shareObjects.angle
    sensorDist = [0,0,0,0]
    
    for i in range(0,4):
      if self.shareObjects.numSamples[i] >  0:
        sensorDist[i] = self.shareObjects.sensorDistance[i]
        self.shareObjects.sensorDistance[i] = 0
        self.shareObjects.numSamples[i] = 0
      else:
        sensorDist[i] = 8190
    self.shareObjects.counter = self.shareObjects.counter +1
    if self.shareObjects.counter > 5: 
      self.shareObjects.distance = 0
      self.shareObjects.offset = 0
      self.shareObjects.angle = 0
    self.shareObjects.lock.release()
    pay = self.payload(code, distance, offset, angle, sensorDist[0], sensorDist[1], sensorDist[2], sensorDist[3], 0)
    try:
      self.shareSerial.serial.write(pay)  # write payload to serial port     
    
    except Exception as e:
      self.shareSerial.isOpen = False
      print(e)
      pass

    self.debugPrint[8] = "Jetson Distance:{0} Offset:{1} Angle:{2} Sensors dist: {3}, {4}, {5}, {6}".format(distance, offset, angle, sensorDist[0], sensorDist[1], sensorDist[2], sensorDist[3])
    logging.warning(self.debugPrint[8])

  def run(self) :
    while True:
      time.sleep(self.rate)
      if self.shareSerial.isOpen:
        self.writeData()

class SerialCarReader(threading.Thread):

  def __init__(self, rate, shareObjects, shareSerial):
    threading.Thread.__init__(self)
    self.shareSerial = shareSerial
    self.rate = rate
    self.shareObjects = shareObjects
    
  def readSerial(self):
      global serialLines
      try:
        c = self.shareSerial.serial.read(10)  
        if len(c) ==  8   and c[0] == 'C' and c[0] == 'O':
          (m1, m2, l, qr, globalState, localState, a, ch) = unpack('BBBBBBBB', c) 
          if m1== 67 and m2 == 79 and (qr + globalState +  localState + a) == ch:  # testing if the message is correct
             self.shareObjects.lock.acquire()
             self.shareObjects.code = qr
             self.shareObjects.globalState = globalState
             self.shareObjects.localState = localState
             self.shareObjects.lock.release()
             logging.warning((m1, m2, l, qr, globalState, localState, a))
             if globalState == 6 and localState == 105:
                    self.shareObjects.seenCode = False
        elif len(c) ==  SIZEOFMAP and c[0] == 'C' and c[0] == 'P':
          (posx, posy, speed, mem, memInPro, memInSync, heading, id, \
              leaderId, OM, tioState, instruction, rssi, prop, Xdistance, \
                YDistance, param1, param2, globalState,boxWidth,boxlenght) = \
                  unpack('ffhHHHhBBBBBBBhhBBBBB', buffer) 
        

        elif len(c) > 0:
          serialLines.put(c)
                 
      except Exception as e:
        print(e)
        self.shareSerial.debugPrint[11] = e
        self.shareSerial.isOpen = False
      

  def run(self) :
    while True:
      time.sleep(self.rate)      
      if self.shareSerial.isOpen:
        self.readSerial()
      else:
        #self.shareSerial.debugPrint[0] = "****Teensy is not connected"
        self.shareSerial.connect()


def main() :    
  speedCamera = 0.1
  speedSerial =  0.005
  speedSensor = 0.015
  myQueue = queue.Queue()
  logging.basicConfig(filename='tioa.log', filemode='w', format='%(message)s')

  
  shareObject = ShareObjects()
  shareObject.code = 0
  shareSerial = ShareSerial()
  distanceSensor = DistanceSensor(speedSensor, shareObject)
  codeReader = CodeReader(speedCamera, shareObject)
  listener = Listener(shareObject)
  serialCarWriter = SerialCarWriter(0.06, shareObject, shareSerial)
  serialCarReader = SerialCarReader(0.03, shareObject, shareSerial)
  readCommand = ReadCommand(shareObject)
  displayRead = DisplaySerial(0.02, shareSerial)
  serialWriterCommand = SerialWriterCommand(0.1, shareObject, shareSerial, myQueue)
  #myAutopilot = autopilot.Autopilot(0.5, myQueue)  

  distanceSensor.start()
  codeReader.start()
  listener.start()
  serialCarWriter.start()  
  serialCarReader.start()
  readCommand.start()
  displayRead.start()
  serialWriterCommand.start()
  #myAutopilot.start()

  #myAutopilot.join()
  serialWriterCommand.join()
  myQueue.join()
  serialCarReader.join()
  serialCarWriter.join()
  distanceSensor.join()
  codeReader.join()
  listener.join()
  readCommand.join()
  displayRead.join()



if __name__ == '__main__':
    main()

