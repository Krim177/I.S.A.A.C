import datetime
import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import json
import io
import subprocess
import threading
import math
import queue

# size in meters of aruco marker
ARUCO_SQUARE_WIDTH = 0.141  # formerly 0.152
CALIB_FILENAME = 'camera_calib.json'
P = 0.85                    # stablization factor 

def gstreamer_pipeline (capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=20, flip_method=2) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))


imgList = queue.Queue(1)
#imgList =[None]
lock = threading.Lock()


# Used to get the latest frame
class CaptureThread(threading.Thread):
    def __init__(self):
        super(CaptureThread, self).__init__()
        #pipeline = "v4l2src device=/dev/video0 ! video/x-raw,width=640,height=512,format=(string)I420,pixel-aspect-ratio=1/1, interlace-mode=(string)progressive, framerate=10/1 ! videoconvert ! appsink"
        #self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2, framerate=30/1), cv2.CAP_GSTREAMER)

        time.sleep(1)

    def run(self):
        while True:
          try:
            if self.cap.isOpened():
                global imgList
                #global lock
                #lock.acquire()
                if not imgList.full():
                   imgList.put(self.cap.read())
                else:
                   temp = self.cap.read()
                #imgList[0] = self.cap.read()
                #if not (imgList[0])[0]:
                #    self.cap.release()
                #    raise SystemExit
                #lock.release()
          except:
             print("Exception cap is not open")
             self.cap.release()
             self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
        
class CodeReader(threading.Thread):
    def __init__(self, rate, shareObjects):
        threading.Thread.__init__(self)
        self.rate = rate
        self.shareObjects = shareObjects
        
        captureThread = CaptureThread()
        captureThread.start()

        self.debugging = False

        #self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)



        # create aruco marker dictionary and get camera calibration
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.camera_mtx, self.dist_coeffs = self.load_camera_calibration(CALIB_FILENAME)
        
    def load_camera_calibration(self, filename):
       with open(filename) as f:
            calib_json = json.load(f)
       camera_mtx = calib_json["camera_matrix"]
       dist_coeffs = calib_json["dist_coeff"]

       return camera_mtx, dist_coeffs        
        
    def debugImage(self, x, z, u_frame, corners, ids, u_camera_mtx, u_dist_coeffs, rvec, tvec, angle):
      z_fmt = 'z: {0} meters'.format(z)
      x_fmt = 'x: {0} meters'.format(x)
      angle_fmt = 'angle: {0} degrees'.format(angle)

      aruco.drawDetectedMarkers(u_frame, corners, ids)
      
      posed_img = aruco.drawAxis(u_frame, u_camera_mtx, u_dist_coeffs, rvec, tvec, 0.1)
      posed_img = cv2.putText(posed_img, z_fmt, (20, 1200), cv2.FONT_HERSHEY_SIMPLEX, 5.0, (0,0,0))
      posed_img = cv2.putText(posed_img, x_fmt, (20, 1350), cv2.FONT_HERSHEY_SIMPLEX, 5.0, (0,0,0))
      posed_img = cv2.putText(posed_img, angle_fmt, (20, 1050), cv2.FONT_HERSHEY_SIMPLEX, 5.0, (0,0,0))
      save_fname = './live_images/' + str(angle)+ 'cAngle_' + str(self.shareObjects.offset) + 'x_' + str(self.shareObjects.distance) + 'z.jpg'
      cv2.imwrite(save_fname, posed_img)

    
    def get_orientation(self, corners):
        #print('Upright, bottom left = 3, bottom right = 2')
        if (corners[0][0][0][1] < corners[0][0][1][1]):
            #left 
            return -1
        else:
            #right
            return 1
        
    def run(self):
        number = 0

        param = aruco.DetectorParameters_create()
        param.cornerRefinementWinSize = 1
        while True:
            #time.sleep(0.008)
            
            global imgList
            image = imgList.get(True)
            #print("read Image", image)
            if self.shareObjects.code > 0:

              #global lock
              #lock.acquire()
              #image = imgList[0][1].copy() #self.cap.read()  # Capture frame
              #lock.release()
              image = image[1].copy()
              if image is not None:
		  #ret, image = self.cap.read()
                  #if ret:
                    
                  #image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                  # typing for open cv
                  u_frame = cv2.UMat(image)

                  ## detect the aruco marker
                  corners, ids, _ = aruco.detectMarkers(u_frame, self.marker_dict, parameters= param)
                  ## no markers were found
                  #print("Corners ", corners)
                  if ids.get() is not None and self.shareObjects.code in ids.get().flatten():
                    ## before we do any work, check if marker with given id is present
                    

                    #print(ids.get())                    
                    ## do conversion to UMat for opencv
                    u_camera_mtx = cv2.UMat(np.array(self.camera_mtx))
                    u_dist_coeffs = cv2.UMat(np.array(self.dist_coeffs))


                    ## get rotation, translation for each single aruco marker
                    rvecs, tvecs, obj_pts = aruco.estimatePoseSingleMarkers(corners, ARUCO_SQUARE_WIDTH, u_camera_mtx, u_dist_coeffs)

                    ## get the index of the marker id we want for the rest of the arrays
                    flat = ids.get().flatten()
                    id_index, = np.where(flat == int(self.shareObjects.code))

                    ## reshape t and r vectors for matrix multiplication
                    rvec = rvecs.get()[id_index][0].reshape((3,1))
                    tvec = tvecs.get()[id_index][0].reshape((3,1))

                    ## Rodrigues baby, look it up, and convert to matrix
                    R, _ = cv2.Rodrigues(rvec)
                    R = np.mat(R).T

                    ## get the camera pose
                    cam_pose = -R * np.mat(tvec)
                    cam_pose = np.squeeze(np.asarray(cam_pose))
                    
                    
                    zVector = (-R) *np.mat([[0, 0, 1]]).T
                    #print("CAmera Pos", -R*np.mat(np.array([0, 0, 1]).T)))

                    ## extract x offset and z camera to marker distance
                    z = cam_pose[-1]
                    x = tvec[0][0]

                    z = round(z, 3)*100
                    x = round(x, 3)*100 + 10 
                    
                    angle = math.atan2(zVector[2][0], zVector[0][0]) * 180/3.141516 - 90

                    #angle =  np.arccos(np.dot(cam_pose, tvec)/(np.linalg.norm(cam_pose)*np.linalg.norm(tvec))) * 180/3.141516
                    #angle = int(self.get_orientation(corners) * angle)
                    
                    self.shareObjects.lock.acquire()
                    ## set to global
                    #print(x, z, angle)
                    #if self.shareObjects.seenCode is False:
                    self.shareObjects.offset = x
                    self.shareObjects.distance = z
                    self.shareObjects.angle = -angle
                    self.shareObjects.seenCode = True
                    self.shareObjects.counter = 0
                    
                    #else:
                    #    self.shareObjects.offset = self.shareObjects.offset*P + (1-P)*x
                    #    self.shareObjects.distance = self.shareObjects.distance*P + (1-P)*z
                    #    self.shareObjects.angle = self.shareObjects.angle*P + (1-P)*-angle
                    ## critical section end
                    self.shareObjects.lock.release()
                    #print("Code {0} - distance: {1} offset:{2} angle:{3} zVector:{4} vector:{5}".format(self.shareObjects.code,z,x, angle, zVector[2][0], zVector[0][0]))

 #                   print('Angle:', int(self.get_orientation(corners) * angle))
                    #print('Corners[0][0]:', corners[0][0])
                    #print('Corner 0:', corners[0][0][0][1])
                    #print('Corner 1:', corners[0][0][1][1])



                    if self.debugging:
                        self.debugImage(x, z, u_frame, corners, ids, u_camera_mtx, u_dist_coeffs, rvec, tvec, angle)
                  #else:
                  #  if self.shareObjects.code > 0 : 
                  #    cv2.imwrite("./live_images/{0}.jpg".format(number), image) 
                  #    number = number + 1


