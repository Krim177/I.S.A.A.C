import threading

class Autopilot(threading.Thread):
    def __init__(self, rate, queue):
        threading.Thread.__init__(self)
        self.rate = rate
        self.queue = queue
        

    #def setCommand(self, ) :
        


    def run(self) :
        while (True) :
            self.queue.put()



# fire up the both producers and consumers
