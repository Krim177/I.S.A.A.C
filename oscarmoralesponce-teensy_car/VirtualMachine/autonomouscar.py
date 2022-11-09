from command import Command
from scipy.spatial.transform import Rotation

PARKING_INIT = 1
PARKING_FIRST_STEP = 2
PARKING_SECOND_STEP = 3
PARKING_DONE = 4


STANDBY = 1
BRAKE = 2
FORWARD = 3
BACKWARD = 4


class AutonomousCar():
    def __init__(self) :
      self.state = PARKING_INIT
      self.steps = 10
      self.rotationAngle = 0
      self.distance = 0
      self.x = 0
      self.y = 0
      

    def round(self, localdynamicMap):
      command = Command(maxTime=500, direction=STANDBY)

      if self.state == PARKING_INIT:
        if self.steps <= 0:
          self.state = PARKING_FIRST_STEP
          self.rotationAngle =  localdynamicMap.heading - 45
          self.x = localdynamicMap.posx - 10
          self.y = localdynamicMap.posy
          self.steps = 10
        else:
          self.steps = self.steps-1
          
      if self.state == PARKING_FIRST_STEP:
        angle = Rotation.from_euler('z', localdynamicMap.heading - self.rotationAngle, degrees=True)
        angle = angle.as_euler('zyx', degrees=True)[0]
        if (angle < 5 and  (localdynamicMap.posx - self.x) <= 2) or self.steps < 0:
          self.state = PARKING_SECOND_STEP
          command.speed = 0
          command.direction = BRAKE
          self.rotationAngle =  localdynamicMap.heading + 45
          self.x = localdynamicMap.posx - 10
          self.y = localdynamicMap.posy
          self.steps = 10
        else:
          command.dir = BACKWARD
          command.orientation = 105
          command.speed = 15
          command.distance = localdynamicMap.posx - self.x
          self.steps = self.steps -1
          

      if self.state == PARKING_SECOND_STEP:
        angle = Rotation.from_euler('z', localdynamicMap.heading - self.rotationAngle, degrees=True)
        angle = angle.as_euler('zyx', degrees=True)[0]
        if (angle > -5 or (localdynamicMap.posx - self.x) <= 2) or self.steps < 0:
          self.state = PARKING_DONE
          command.speed = 0
          command.direction = BRAKE
        else:
          command.dir = BACKWARD
          command.orientation = 75
          command.speed = 15
          command.distance = localdynamicMap.posx - self.x
          self.steps = self.steps - 1
          

      if self.state == PARKING_DONE:
          command.orientation = 0
          command.speed = 0
          command.direction = STANDBY


      #print(self.state, localdynamicMap.heading)
      return command        

  
  
  


