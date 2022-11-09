import socket
import sys
import os
import threading
import subprocess


HOST_IP = ""
HOST_PORT = 12583                    #Random port
MAX_CONN = 2

sock = socket.socket()               #initialize the socket
sock.bind((HOST_IP,HOST_PORT))       #bind the ip and port to the socket
sock.listen(MAX_CONN)                #have the socket listen to only MAX_CONN connections


class fileServer(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        
    while True:
        (comm, addr) = sock.accept()     #Waits for a connection

        #Open the file reading it in binary
        f = open("/home/user/teensy_car/main.hex",'wb') 

        l = comm.recv(1024)

        while (l):
            f.write(l)
            l = comm.recv(1024)

        f.close()
        comm.close()
        subprocess.Popen(['sh',"../upload.sh"]);
        print("main.hex upload has been completed")
    sock.close()
    
if __name__ == '__main__':
    server = fileServer();
    server.start()
    server.join()
    
