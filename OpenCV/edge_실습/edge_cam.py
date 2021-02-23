#!/usr/bin/env python
# -*- coding: utf-8 -*-

#OpenCV 사용 및 numpy 사용 준비

import cv2
import rospy
import numpy as np 

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#ROS에서 OpenCV를 편하게 사용하기 위한 Cv_Bridge사용 준비
bridge = CvBridge()
cv_image = np.empty(shape=[0])


#Image 토픽을 처리하는 콜백함수 정의 
def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

#노드를 생성하고 Image로 부터 토픽이 오면 콜백함수가 처리
rospy.init_node('cam_tune', anonymous=True)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

while not rospy.is_shutdown():
    
    #640x480이미지 한 장이 모일 때까지 잠시 기다린다.
    if cv_image.size != (640*480*3):
        continue

    #원본 이미지를 그레이칼라로 변경 
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #부드럽게 변경
    blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)

    #이미지 외곽선만 표시하게 변경
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 70)
    

    #원본 포함 4개의 이미지를 표시 
    cv2.imshow("original", cv_image)
    cv2.imshow("gray", gray)
    cv2.imshow("gaussian blur", blur_gray)
    cv2.imshow("edge", edge_img)
    cv2.waitKey(1)

