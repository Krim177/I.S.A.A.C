import time
import threading
import shareobjects
import time
import VL53L0X

  
class Sensors(threading.Thread):
  def __init__(self, rate, shareObjects):
    threading.Thread.__init__(self)
    self.rate = rate
    self.shareObjects = shareObjects
  
  def setup(self):
    print("Thread started: getting sensor distance")
    self.tof0 = VL53L0X.VL53L0X(TCA9548A_Num=0, TCA9548A_Addr=0x70)
    self.tof1 = VL53L0X.VL53L0X(TCA9548A_Num=1, TCA9548A_Addr=0x70)
    self.tof2 = VL53L0X.VL53L0X(TCA9548A_Num=2, TCA9548A_Addr=0x70)

    self.tof0.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    self.tof1.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    self.tof2.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    

  def run(self):
    distance = [0,0,0]
    self.setup()
    
    while True:
      time.sleep(self.rate)
      
      distance0 = self.tof0.get_distance() / 10
      distance1 = self.tof1.get_distance() / 10
      distance2 = self.tof2.get_distance() / 10
      self.shareObjects.lock.acquire(True)
      self.shareObjects.sensorDistance0 = int(distance0)
      self.shareObjects.sensorDistance1 = int(distance1)
      self.shareObjects.sensorDistance2 = int(distance2)
      self.shareObjects.lock.release()
      #print("Distance 0", distance0)
      #print("Distance 1", distance1)

    self.tof0.stop_ranging()
    self.tof1.stop_ranging()
    self.tof2.stop_ranging()

  

  
