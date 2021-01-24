#!/usr/bin/env python

import rospy, math, os
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler


rospy.init_node('imu_generator')
pub=rospy.Publisher('imu', Imu, queue_size=10)
rate=rospy.Rate(10)


f=open("/home/soorim/xycar_ws/src/rviz_imu/src/imu_data.txt",'r')
lines=f.readlines()

imuMsg = Imu()
imuMsg.header.frame_id = 'map'
seq=0

for line in lines:

	
	item=line.split(",")
	
	roll=float(item[0][7:21])
	pitch=float(item[1][9:23])
	yaw=float(item[2][7:21])
	
	quat =quaternion_from_euler(roll,pitch,yaw)	
	
	
	imuMsg.orientation.x = quat[0]
        imuMsg.orientation.y = quat[1]
        imuMsg.orientation.z = quat[2]
        imuMsg.orientation.w = quat[3]
	
        imuMsg.header.stamp= rospy.Time.now()

        imuMsg.header.seq = seq
        
        seq = seq + 1
	
	pub.publish(imuMsg) 
	rate.sleep()
  	
  
