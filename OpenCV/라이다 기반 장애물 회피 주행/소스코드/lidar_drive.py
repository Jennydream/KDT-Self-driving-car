#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_motor.msg import xycar_motor

motor_msg = xycar_motor()
distance = []

def callback(data):
    global distance, motor_msg
    distance = data.ranges


def drive_go(speed, angle):
    global motor_msg
    motor_msg.speed = speed
    motor_msg.angle = angle
    pub.publish(motor_msg)

def drive_stop():
    global motor_msg
    motor_msg.speed = 0
    motor_msg.angle = 0 
    pub.publish(motor_msg)

	
def drive_avoidance():
    for i in range(10):
	drive_go(5, -30)
    	time.sleep(0.1)

    for i in range(30):
	drive_go(5, 30)
    	time.sleep(0.1)
    


rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)


time.sleep(3) #ready to connect lidar 

while not rospy.is_shutdown():
    ok = 0
    for degree in range(60,120):
        if distance[degree] <= 0.3:
            ok += 1
        if ok > 3:
	    drive_stop()
	    time.sleep(1)
	    for back in range(50):         
	        drive_go(-5,0)
		time.sleep(0.1)
	    
	    time.sleep(1)          
            drive_avoidance() 	
  
	    break
        	
                      
    if ok <= 3:
        drive_go(5,0)






