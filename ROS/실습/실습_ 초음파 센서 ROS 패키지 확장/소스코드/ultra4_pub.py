#!/usr/bin/env python

import serial, time, rospy, re
from std_msgs.msg import Int32MultiArray


ser_front = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    )

def read_sensor():
    serial_data = ser_front.readline()
   
    ser_front.flushInput()
    ser_front.flushOutput()    
    

    ultra4=serial_data.split(" ")

    data=[]

    for i in range(4):
	ultrasonic_data = int(filter(str.isdigit, ultra4[i]))
	data.append(ultrasonic_data)
  
    msg.data=data

  
  
if __name__ == '__main__':

    rospy.init_node('ultra4_pub', anonymous=False) # initialize node
    pub = rospy.Publisher('ultra4', Int32MultiArray, queue_size=1)
    

    msg = Int32MultiArray() # message type
    
    while not rospy.is_shutdown():
        read_sensor() 
        pub.publish(msg) # publish a message
        time.sleep(0.2)
    
    ser_front.close()



