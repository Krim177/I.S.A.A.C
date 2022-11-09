from threading import Semaphore
import copy

class LocalDynamicMap() :
    def __init__(self):
      self.localDynamicMap = None
      self.semaphore = Semaphore()

    def update(self, localMap) :
      self.semaphore.acquire(True)
      #self.localDynamicMap[localMap.id] =  localMap
      self.localDynamicMap =  localMap
      self.semaphore.release()

    def get(self) :
      self.semaphore.acquire(True)
      localMap = copy.deepcopy(self.localDynamicMap)
      self.semaphore.release()
      return localMap


class LocalMap:
    def __init__(self, posx, posy, speed, mem, memInPro, memInSync, heading, id, \
              leaderId, OM, tioState, instruction, rssi, prop, Xdistance, \
                YDistance, param1, param2, globalState, boxWidth,boxlenght, t) :
      self.posx = posx
      self.posy = posy
      self.speed = speed
      self.mem = mem
      self.memInPro = memInPro
      self.memInSync = memInSync
      self.heading = heading
      self.id = id
      self.leaderId = leaderId
      self.OM = OM
      self.tioState = tioState
      self.instruction = instruction
      self.rssi = rssi
      self.prop = prop
      self.Xdistance = Xdistance
      self.YDistance = YDistance
      self.param1 = param1
      self.param2 = param2
      self.globalState = globalState
      self.boxWidth = boxWidth
      self.boxlenght = boxlenght
      self.t = t
      


  
  
  


