import threading
import time
import serial
from struct import *
import array
import sys
import signal



class carController(threading.Thread):

  def __init__(self, ):
    threading.Thread.__init__(self)
    self.serial = None
    self.porti = 0
    self.speed = 0
    self.orientation = 0
    self.distance = 0
    self.direction = 0

  def boundUnsigned(self, x) :
      if x > 2000: 
          x = 2000
      if x < 0:
          x = 0
      return x
  
  def boundSigned(self, x) :
      if x > 2000: 
          x = 2000
      if x < -2000:
          x =-2000
      return x
 

  def payload(self, speed, orientation, distance, direction):
    ch = 0
    length = 14
    pay = pack('hhhhBBI', self.boundSigned(speed), self.boundSigned(int(orientation)), self.boundSigned(int(distance)), 
            self.boundSigned(int(direction)), 1, 0)
    a = array.array('b', pay).tolist()
    for i in range(0, length):
      ch = (ch + int(a[i])) % 256

    pay = pack('BBB', 67, 79, length) + pay + pack('B', ch)
    return pay
    
  def connect(self) :   
    try:
       self.serial = serial.Serial(port='/dev/ttyACM{0}'.format(self.porti), baudrate=115200, timeout=0)
       
    except:
       self.porti = (self.porti + 1) % 5
       pass
      
  def readSerial(self):
    
      readLines = False
      lines = []
      try:
        lines = self.serial.read(150).splitlines()     
        readLines = True
      except:
        #print("Error in reading")
        self.connect()
      if readLines:
        for c in lines:
           if len(c) == 8:
              (m1, m2, l, qr, globalState, localState, a, ch) = unpack('BBBBBBBB', c) 
              if m1== 67 and m2 == 79 and (qr + globalState +  localState + a) == ch:  # testing if the message is correct
                 self.shareObjects.lock.acquire(True)
                 self.shareObjects.code = qr
                 self.shareObjects.globalState = globalState
                 self.shareObjects.localState = localState
                 self.shareObjects.lock.release()
                 if globalState == 6 and localState == 105:
                    self.shareObjects.seenCode = False
           else:
             print("Teensy {0}".format(c.decode("utf-8")))
      
      

  def writeData(self):
    
      pay = self.payload(self.speed, self.orientation, self.distance, self.direction)
      try:
        self.serial.write(pay)  # write payload to serial port      
      except:
        #print("Error")
        self.connect()

  def run(self) :
    while True:
      time.sleep(50)
      self.readSerial()
      self.writeData()

def main() :    
  carcontroller = carController()
  carcontroller.start()      



if __name__ == '__main__':
    main()

