import threading

class ShareObjects():
  def __init__(self):
    self.lock = threading.Lock()
    self.code = 0
    self.distance =  0
    self.offset = 0
    self.angle = 0
    self.sample = 0
    self.sensorDistance =[0, 0, 0, 0, 0]
    self.numSamples =[0, 0, 0, 0, 0]
    self.seenCode = False
    self.counter = 0
