import socket
import sys
import os

HOST_PORT = 12583                    #Random port

if __name__ == "__main__":
    ip = sys.argv[1]

    sock = socket.socket()               #initialize the socket
    sock.connect((ip , HOST_PORT))       #bind the ip and port to the socket
            
    f = open("main.hex", "rb")

    l = f.read(1024)
    
    while (l):
        sock.send(l)
        l = f.read(1024)
    sock.close()
