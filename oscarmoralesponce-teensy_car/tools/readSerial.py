import serial
import serial.tools.list_ports
import time

class ReaderSerial:
  def __init__(self) :
    self.connect()

  def  connect(self):
    comlist = serial.tools.list_ports.comports()
    for p in comlist:
      try:
        if 'ttyACM' in p[0] or 'COM' in p[0] :    
          self.ser = serial.Serial(port=p[0], baudrate=115200)
          print(f"Connected  to {p[0]}")
          break
      except Exception as e:
        pass
        print(e)

  def read(self, substr):
    while 1: 
        try:
          c = self.ser.read_all()
          if len(c) > 0:
            c1 = c.decode("utf-8", "replace")
            if substr in c1:
              print(c1)
        except Exception as e:
          #print(e)
          time.sleep(0.1)
          self.connect()

readerSerial = ReaderSerial()
readerSerial.read("Test")
