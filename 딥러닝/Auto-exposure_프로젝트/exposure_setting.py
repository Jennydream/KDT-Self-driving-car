#!/usr/bin/env python
import os
import cv2,csv,time
import sys
import rospy, rospkg
import signal
import numpy as np

from Tkinter import Tk
import tkFileDialog as filedialog
import xml.etree.ElementTree as elemTree
#from PIL import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor


device_dir = "/dev/videoCAM"

cv_image = np.empty(shape=[0])
bridge = CvBridge()



video_name = "/home/nvidia/Desktop/test.mkv"
csv_name = "/home/nvidia/Desktop/test.csv"


#make csv file
out = cv2.VideoWriter(video_name,cv2.VideoWriter_fourcc('M','J','P','G'), 10, (640, 480))
f = open(csv_name, 'w')
wr = csv.writer(f)
wr.writerow(["ts_micro", "frame_index", "threshold"])



def camera_callback(data):
    global cv_image
    global bridge
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

def drive(Angle, Speed):
    global pub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)

def exposure_set(value):
    global device_dir

    if str(type(value)) != "<type 'int'>":
        return

    command = "v4l2-ctl -d "+ device_dir +" -c exposure_absolute="
    command += str(value)
    os.system(command)
    

def exposure_get():
    global device_dir


    command = "v4l2-ctl -d "+ device_dir +" -C exposure_absolute"
    #command = "v4l2-ctl -d "+ device_dir +" -c exposure_auto=3"
    v= os.system(command)
    return v

rospy.init_node("exposure_change", anonymous=True)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

rospy.Subscriber("/usb_cam/image_raw/", Image, camera_callback)
#cam = cv2.VideoCapture(device_dir, cv2.CAP_V4L2)



cap = cv2.VideoCapture(0)

cnt=5
ex=0
fre=0

while True:

    if cv_image.size !=(640*480*3):
	continue
      
    #set exposure from 5 to 70 
    if cnt>=70:
	cnt=5
    exposure_set(cnt)
    
    va=exposure_get()

    #make csv file
    wr.writerow([time.time(), fre , cnt])
    out.write(cv_image)
    

    cv2.imshow('xytron', cv_image)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break  
    #drive(0,15)

    fre+=1
    cnt+=1
cam.release()
cv2.destroyAllWindows()



