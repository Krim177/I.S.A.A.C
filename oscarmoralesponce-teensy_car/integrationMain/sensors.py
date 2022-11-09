import time
import threading
import shareobjects
import time

error = False

try:
  import RPi.GPIO as GPIO
  import VL53L0X
except:
  print("No Sensor VL53L0X detected ")
  error = True
  
  class Sensors(threading.Thread):
     def __init__(self, shareObjects):
       threading.Thread.__init__(self)
       self.shareObjects = shareObjects
   
     def getDistanceSim(self) : 
        distance = input("Distance: ")
        return int(distance)
      
     def setup(self):
       print("Thread started: getting sensor distance")
       self.tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
       self.tof.open()
  
     def run(self):
        distance = [0,0,0]
        self.setup()
        self.tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
        while True:
          time.sleep(0.05)
          
          # timing = tof.get_timing()
          #if timing < 20000:
          #   timing = 20000
          distance = self.tof.get_distance()
          self.shareObjects.lock.acquire(True)
          self.shareObjects.sensorDistance1 = distance
          #print("Distance", distance/10)
          #print("getDistance", self.shareObjects.sensorDistance1)
          self.shareObjects.lock.release()
          
        

        tof.stop_ranging()

  
  
  class Sensors(threading.Thread):
     def __init__(self, shareObjects):
       threading.Thread.__init__(self)
       self.shareObjects = shareObjects
       
     def getDistance(self) : 
        distance = input("Distance: ")
        return int(1)
     
     def run(self):
        distance = [0,0,0]
        while True:
          time.sleep(1)
          '''
          distance[0] = self.getDistance()
          if self.shareObjects.lock.acquire(True): 
             self.shareObjects.sensorDistance1 = distance[0]
             #print("getDistance", self.shareObjects.sensorDistance1)
             #self.shareObjects.sensorDistance2 = distance[1]
             #self.shareObjects.sensorDistance3 = distance[2]
             self.shareObjects.lock.release()
          '''

  
