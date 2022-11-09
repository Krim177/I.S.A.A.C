import threading
import time
import serial
import serial.tools.list_ports as port_list
from struct import *
import array
import asyncio, random

from shareobjects import ShareObjects  
from listener import Listener
from sensors import Sensors
from codereader import CodeReader

queue = asyncio.Queue()
serialConnection = None

def connect() :   
  global serialConnection
  ports = list(port_list.comports())
  for p in ports:
    try:
        if 'ttyACM' in p[self.porti] or 'COM' in p[self.porti] :  
          serialConnection = serial.Serial(port=p[self.porti], baudrate=115200)
          print(f"Connected  to {p[self.porti]}")
    except:
        self.porti = (self.porti + 1) % 4
        pass
    


class SerialReader(threading.Thread):
    def __init__(self, rate, shareObjects):
      threading.Thread.__init__(self)
      self.serial = None
      self.rate = rate
      self.shareObjects = shareObjects


class SerialCar(threading.Thread):

  def __init__(self, rate, shareObjects):
    threading.Thread.__init__(self)
    self.serial = None
    self.rate = rate
    self.shareObjects = shareObjects
    
  def payload(self, code, distance, offset, sensor1, sensor2, sensor3):
    ch = 0
    length = 12
    pay = pack('HhHHH', 1000, code, distance, offset, sensor1, sensor2, sensor3)
    a = array.array('b', pay).tobytes()
    for i in range(0, 12):
      ch = (ch + a[i]) % 256

    pay = pack('BBB', 67, 79, 12) + pay + pack('B', ch)
    return pay
    
  def connect(self) :   
    ports = list(port_list.comports())
    for p in ports:
      try:
        if 'ttyACM' in p[0] or 'COM' in p[0] :  
          self.serial = serial.Serial(port=p[0], baudrate=115200)
          print(f"Connected  to {p[0]}")
      except:
        pass
      
  def readSerial(self):
      try:
           c = self.serial.readline()
           if len(c) == 6:
              (m1, m2, l, qr, command, ch) = unpack('BBBBBB', c) 
              if m1== 67 and m2 == 79 and (qr + command) == ch:  # testing if the message is correct
                 self.shareObjects.lock.acquire(True)
                 self.shareObjects.code = qr
                 self.shareObjects.lock.release()
           else:
             print("Teensy ", c.decode("utf-8"))
      except Exception as e:
        self.connect()
      
      

  def writeData(self):
    
      try:
        self.shareObjects.lock.acquire(True) 
        #print("Sending to serial ", self.shareObjects.code, self.shareObjects.sensorDistance1)
        pay = self.payload(self.shareObjects.code, int(self.shareObjects.distance), int(self.shareObjects.offset), self.shareObjects.sensorDistance1, self.shareObjects.sensorDistance2, self.shareObjects.sensorDistance3)
        self.shareObjects.lock.release()
        self.serial.write(pay)  # write payload to serial port      
      except:
        self.connect()

  def run(self) :
    while True:
      time.sleep(self.rate)
      self.readSerial()
      #self.writeData()
      

'''
shareObject = ShareObjects()
sensors = Sensors(shareObject)
listener = Listener(shareObject)
codeReader = CodeReader(0.5, shareObject)
serialCar = SerialCar(0.05, shareObject)
serialReader = SerialReader(0.01, shareObject)
sensors.start()
listener.start()
codeReader.start()
serialCar.start()      
'''


 
async def serialReader(queue, rate):
    global serialConnection
    while True:
        # produce a token and send it to a consumer
        try:
          c = serialConnection.readline()
          c = serialLines.get()
          await queue.put(c)
          await asyncio.sleep(rate)
        except:
          connect()
 
async def consumer(queue, rate):
    while True:
        c = await queue.get()
        # process the token received from a producer
        queue.task_done()
        
        if len(c) == 6:
          (m1, m2, l, qr, command, ch) = unpack('BBBBBB', c) 
          if m1== 67 and m2 == 79 and (qr + command) == ch:  # testing if the message is correct
              self.shareObjects.lock.acquire(True)
              self.shareObjects.code = qr
              self.shareObjects.lock.release()
          
        else:
          try:
            print(len(c), c.decode("utf-8"))
          except Exception as e:
            pass
        
        await asyncio.sleep(rate)
 
async def main():
    queue = asyncio.Queue()
 
    # fire up the both producers and consumers
    producers = asyncio.create_task(serialReader(queue, 0.01))
                 
    consumers = asyncio.create_task(consumer(queue, 0.01))
                 
    # with both producers and consumers running, wait for
    # the producers to finish
    await asyncio.gather(producers)
    print('---- done producing')
 
    # wait for the remaining tasks to be processed
    await queue.join()
 
    # cancel the consumers, which are now idle
    for c in consumers:
        c.cancel()
 
asyncio.run(main())