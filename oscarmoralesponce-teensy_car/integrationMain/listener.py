import socket
import threading
from struct import *
from time import sleep

class Listener(threading.Thread):

    def __init__(self, shareObjects):
        threading.Thread.__init__(self)
        self.shareObjects = shareObjects

    def run(self):
        print("Code listener running")
        while True:
           sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)             # UDP socket
           sock.bind(('', 2000))                                               # Bind to localhost on port 2000

           data, senderAddr = sock.recvfrom(6, 0)                           # Receive broadcast, buffer size 1500
           #data = int( data.decode('UTF-8') )
           print(data)
           (code, distance, offset) = unpack("hhh", data)
           print("Broadcast code received: ", code, distance, offset)
           if self.shareObjects.lock.acquire(True):
              self.shareObjects.code = code
              self.shareObjects.distance = distance
              self.shareObjects.offset = offset
           self.shareObjects.lock.release()
           print("shareObjects set")