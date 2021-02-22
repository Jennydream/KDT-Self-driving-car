#!/usr/bin/env python

import rospy ,time, serial
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range


lidar_points= None

def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges
    
  


rospy.init_node("lidar")
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)



pub1 = rospy.Publisher('scan1', Range, queue_size=1)
pub2 = rospy.Publisher('scan2', Range, queue_size=1)
pub3 = rospy.Publisher('scan3', Range, queue_size=1)
pub4 = rospy.Publisher('scan4', Range, queue_size=1)

msg=Range()
h=Header()

msg.header=h
msg.radiation_type = Range().ULTRASOUND
msg.field_of_view = (20.0/180.0)*3.14
msg.min_range = 0.2
msg.max_range = 2.0



while not rospy.is_shutdown():
	
	if lidar_points==None:
		continue

		
	for i in range(len(lidar_points)):
		
		h.frame_id="front"
        	msg.range =lidar_points[i]
        	pub1.publish(msg)
		
	for i in range(len(lidar_points)):
		h.frame_id="back"
        	msg.range =lidar_points[i]
		pub2.publish(msg)
		
	for i in range(len(lidar_points)):
		h.frame_id="left"
       	 	msg.range =lidar_points[i]
        	pub3.publish(msg)
		
	for i in range(len(lidar_points)):

		h.frame_id="right"
        	msg.range =lidar_points[i]	
        	pub4.publish(msg)
	
	

       


