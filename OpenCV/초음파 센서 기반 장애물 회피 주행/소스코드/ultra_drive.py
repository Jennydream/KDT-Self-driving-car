#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_motor.msg import xycar_motor

motor_msg = xycar_motor()
distance = []
t = 3
back = rospy.Duration(3)
inflection_1st = rospy.Duration(t)
turning_max = inflection_1st + rospy.Duration(t)
inflection_2nd = turning_max + rospy.Duration(t)
returnning = inflection_2nd + rospy.Duration(t)

angle_traj = 0



def callback(data):
    global distance
    distance = data.data

def drive_go(speed, angle):
    global motor_msg, pub
    motor_msg.speed = speed
    motor_msg.angle = angle
    pub.publish(motor_msg)

def drive_stop():
    global motor_msg, pub
    motor_msg.speed = 0
    motor_msg.angle = 0 
    pub.publish(motor_msg)

def drive_avoid(speed, angle):
    for i in range(30):
	drive_go(speed, -angle)
    	time.sleep(0.1)

    for i in range(40):
	drive_go(speed, angle)
    	time.sleep(0.1)
    


rospy.init_node('ultra_driver')
rospy.Subscriber('/xycar_ultrasonic', Int32MultiArray, callback, queue_size = 1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

time.sleep(3) #ready to connect lidar 



while not rospy.is_shutdown():
    
    if distance[1] <= 20 or distance[2] <= 20 or distance[3] <= 20:

	drive_stop()
	time.sleep(1)

	for i in range(100):
	    drive_go(-17, 0)
	    time.sleep(0.1)

        if distance[1] >= distance[3]:
	    drive_avoid(15,30)	
	else:
	    drive_avoid(15,-30)

    else:
	drive_go(25, 0)
  

