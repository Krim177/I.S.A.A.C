import threading
import time


class VirtualMachine(threading.Thread):
    def __init__(self, rate, localDynamicMap, algo, commandQueue) :
      threading.Thread.__init__(self)
      self.rate = rate
      self.localDynamicMap = localDynamicMap
      self.algo = algo
      self.commandQueue = commandQueue

    def run(self):
      command = ""
      while True:
        time.sleep(self.rate)
        localMap = self.localDynamicMap.get()
        if localMap is not None:
          command = self.algo.round(localMap)
          self.commandQueue.put(command)

        

  
  
  


