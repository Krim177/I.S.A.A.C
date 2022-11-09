import threading

class ShareObjects():
  def __init__(self):
    self.lock = threading.Lock()
    self.code = 0
    self.distance =  0
    self.offset = 0
    self.sensorDistance1 = 0
    self.sensorDistance2 = 0
    self.sensorDistance3 = 0