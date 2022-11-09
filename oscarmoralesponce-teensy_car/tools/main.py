import threading
from time import sleep
import serial

from aruco_calculations import ArucoCalculator
from codeListener import codeListener

#global offset
#global distance
#global condition

# Global variables
condition = threading.Condition()
printSem = threading.Semaphore()
serialSem = threading.Semaphore()

# size in meters of aruco marker
ARUCO_SQUARE_WIDTH = 0.141  # formerly 0.152
CALIB_FILENAME = 'camera_calib.json'


if __name__ == "__main__":
    seconds_per_capture = 0.5                                             # Seconds per capture for Pi Camera

    cameraThread = ArucoCalculator(seconds_per_capture, condition)                 # Create process threads
    teensyInterface = codeListener(serialSem, printSem, condition)

    cameraThread.start()                                                # Start threads
    teensyInterface.start()

#    readThread = threading.Thread(target=readSerial())                  # Create thread to read serial
