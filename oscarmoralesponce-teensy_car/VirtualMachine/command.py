
FORWARD = 1
BACKWARD = 2

class Command() :
    def __init__(self, maxTime=0, speed=0, orientation=90, distance=0, direction=1, stop = 1):
      self.maxTime = maxTime
      self.speed = speed
      self.orientation = orientation
      self.distance = distance
      self.orientation = orientation
      self.direction = direction
      self.stop = stop
