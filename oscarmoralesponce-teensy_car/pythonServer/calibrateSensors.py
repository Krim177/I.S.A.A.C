import serial
import re
import time
import numpy


reading = []

class DistanceSensor:
  def __init__(self):
    self.serial = None
    self.porti = 0
    self.line = ""

    
  def connect(self) :   
    try:
       self.serial = serial.Serial(port='/dev/ttyUSB{0}'.format(self.porti), baudrate=9600, timeout=0.01)        
    except Exception as e:
       self.porti = (self.porti + 1) % 4
       print("Connect", e, self.porti)
      
  def readSerial(self):
      c = ""
      try:
          c = self.serial.readline()
      except:
          #print("READ", e, self.serial)
          self.connect()
      data = c.split(",")
      print(data)
      if len(data) == 6:
        if data[0] == "CO" and 'X' in data[5]:
          for i in range(0, 4):
            if int(data[i+1]) < 8190: 
              reading[i].append(int(data[i+1]))
          return True
        return False

        
distanceSensor = DistanceSensor()
reading.append([])
reading.append([])
reading.append([])
reading.append([])


while True:
    time.sleep(0.033)
    if distanceSensor.readSerial():
      if len(reading[0])  > 100 and len(reading[0]) > 100: 
       break
     
     
with open("sensorcalibration.txt", 'w') as f:
    for i in range(0, 4):
      a = numpy.array(reading[i], numpy.int32)
      #print(numpy.std(a))
      #print(numpy.average(a) - 150)
      if numpy.average(a) != 0:
        offset = str(int(numpy.average(a)))
        print(offset)
        f.write(offset)
        f.write("\n")



