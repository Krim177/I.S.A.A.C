import threading
import serial
import serial.tools.list_ports as port_list
import re
import time
from struct import *
import array


from shareobjects import ShareObjects
INIT = 0
SECOND = 1
LENGTH = 2
DATA = 3
CHECKSUM = 4



class DistanceSensor(threading.Thread):
  def __init__(self, rate, shareObjects):
    threading.Thread.__init__(self)
    self.serial = None
    self.rate = rate
    self.shareObjects = shareObjects
    self.porti = 0
    self.line = ""
    #self.endTime = 0
    #self.startTime = 0
    offset = tuple(open("sensorcalibration.txt", 'r'))
    self.offset = []
    for c in offset : 
      self.offset.append(int(c))
    for i in range(0, 4): 
      self.offset.append(0)

    #print(self.offset[0])

    
  def connect(self) :  
    ports = list(port_list.comports()) 
    for p in ports:
      try:
        if 'ttyACM' in p[0] or 'COM' in p[0] :  
          serialConnection = serial.Serial(port=p[0], baudrate=115200)
          print(f"Connected  to {p[0]}")
      except:
        self.porti = (self.porti + 1) % 4
      
  def readSerial(self):
      c = ""
      try:
          c = self.serial.readline()
          
      except:
          #print(e)
          self.connect()
          
      data = c.split(",")
      
      if len(data) == 6:
        if data[0] == "CO" and 'X' in data[5]:
          #print(data)
          self.shareObjects.lock.acquire()
          for i in range(0, 4):
            if int(data[i+1]) < 8190:   
              self.shareObjects.sensorDistance[i] = self.shareObjects.sensorDistance[i] + int(data[i+1]) - self.offset[i]
              self.shareObjects.numSamples[i] = self.shareObjects.numSamples[i] + 1
              
            #else:
	          #print(i, "Failure")
          self.shareObjects.lock.release()      
      
  def run(self) :
    while True:
      time.sleep(self.rate)
      self.readSerial()

        
#distanceSensor = DistanceSensor(0, 0)
#distanceSensor.start()     
#while True:
#    time.sleep(0.005)
#    distanceSensor.readSerial()
