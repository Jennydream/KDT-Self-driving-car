#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2, time
from sensor_msgs.msg import Image
from xycar_motor.msg import xycar_motor
from cv_bridge import CvBridge

import torch
import torch.nn as nn
import torch.optim as optim

import rospkg
from model import end2end

import glob, random, time, io, dill, os, cv2

import numpy as np



class Autoexposure():
    
    def __init__(self):
	
	self.device_dir = "/dev/videoCAM"
	self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
	self.before_value=0
    
    def study_model_load(self, episode, batch_cnt, model, device):
	rospack = rospkg.RosPack()
	LoadPath_main = rospack.get_path('white2')+"/src/save/main_model_"+str(episode).zfill(6)+"_"+str(batch_cnt).zfill(6)+ ".pth"
        with open(LoadPath_main, 'rb') as f:
            LoadBuffer = io.BytesIO(f.read())
            model.load_state_dict(torch.load(LoadBuffer, map_location=device))
    	return model

    def exposure_set(self,value):
	
        if str(type(value)) != "<type 'int'>":
            return
        if value == self.before_value:
            return
        else:
            self.before_value=value
	command = "v4l2-ctl -d "+ self.device_dir +" -c exposure_absolute="
	command += str(value)
	os.system(command)
    
    def change_exposure(self,cv_image):
	net = end2end().to(device)
	net = self.study_model_load(1000, 77, net, device)
    
	frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)
        frame = cv2.resize(frame, dsize=(200, 112))
    
        frame = frame[46:, :]
  
        frame = frame.transpose((2, 0, 1)) / 255.0
        t_frame = torch.FloatTensor([frame]).to(device)
    
        ex=net(t_frame)
       
        ex_value=int(ex.tolist()[0][0])
        #print("current : ",ex_value)    

        result_ex=46

        if ex_value<30:
	    result_ex=50
        if ex_value>=60:
	    result_ex=40

        
        self.exposure_set(result_ex)
        
    
        #print("after change",result_ex)
    
	

    
