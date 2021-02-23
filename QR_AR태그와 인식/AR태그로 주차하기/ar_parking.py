#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray

xycar_msg = Int32MultiArray()

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
angle=0
speed=0

def callback(msg):
    global arData,motor_pub

   
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

rospy.init_node('ar_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size =1 )
xycar_msg = Int32MultiArray()

def back(angle, num):
	global xycar_msg, motor_pub

	
	for _ in range(num):
		xycar_msg.data=[angle,-10]
		motor_pub.publish(xycar_msg)
		time.sleep(0.1)

	for _ in range(num/2):
		xycar_msg.data=[-angle,-10]
		motor_pub.publish(xycar_msg)
		time.sleep(0.1)



while not rospy.is_shutdown():

    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"], arData["AY"],
                                            arData["AZ"], arData["AW"]))
	
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    print("=======================")
    print(" roll  : " + str(round(roll,1)))
    print(" pitch : " + str(round(pitch,1)))
    print(" yaw   : " + str(round(yaw,1)))

    print(" x : " + str(round(arData["DX"],0)))
    print(" y : " + str(round(arData["DY"],0)))
    print(" z : " + str(round(arData["DZ"],0)))

	
    img=np.zeros((100,500,3))
    cv2.line(img,(20,40),(20,80),(0,0,255),2)
    cv2.line(img,(230,40),(230,80),(0,0,255),2)
    cv2.line(img,(480,40),(480,80),(0,0,255),2)
    cv2.line(img,(20,60),(480,60),(0,0,255),2)


    point=int(arData["DX"])+240
	
    if point >480:
	point=480
    elif point <20:
	point=20

    img=cv2.circle(img,(point,60),15,(0,255,0),-1) 

    distance=math.sqrt(pow(arData["DX"],2)+pow(arData["DY"],2))

    cv2.putText(img,str(int(distance)),(450,25),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255))


    
    dx_dy_yaw="DX :"+str(round(arData["DX"],0))+" DY :"+str(round(arData["DY"],0))+" Yaw :"+str(round(yaw,1))
  
    cv2.putText(img,dx_dy_yaw,(20,25),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255) )

    cv2.imshow('AR Tag Position',img)
    cv2.waitKey(1)


    if (arData["DY"]!=0):
	
    	angle=(math.atan(arData["DX"]/arData["DY"]))*180/math.pi
	
    if int(arData["DY"]>150):
	speed=25
    elif int(arData["DY"]>10):
	 
	speed=15
        
        if 40<int(arData["DY"])<68:
		speed=0
	if (yaw>10):
		back(-50,30)
	elif (yaw<-10):

		back(50,30) 
	

  

    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)
	
    

cv2.destroyAllWindows()

