#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch
import torch.nn as nn
import torch.optim as optim

from model import end2end

import glob, csv, random, time, io, dill, os, cv2

import numpy as np
from PIL import Image


def study_model_load(episode, batch_cnt, model, device):

    
    LoadPath_main = os.getcwd()+"/save/main_model_"+str(episode).zfill(6)+"_"+str(batch_cnt).zfill(6)+ ".pth"
   
    with open(LoadPath_main, 'rb') as f:
        LoadBuffer = io.BytesIO(f.read())
   
    model.load_state_dict(torch.load(LoadBuffer, map_location=device))
    
    
    return model
    
    


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

net = end2end().to(device)

net = study_model_load(1700,42, net, device)

input_file = "0x7f0961a722d0.mkv"

cap = cv2.VideoCapture(input_file)
 

while cap.isOpened():
    retval, oframe = cap.read()

    if not(retval):
        break 

    frame = cv2.cvtColor(oframe, cv2.COLOR_BGR2YUV)
    frame = cv2.resize(frame, dsize=(200, 112))
    frame = frame[46:,:]
    frame = frame.transpose((2, 0, 1)) / 255.0
    t_frame = torch.FloatTensor([frame]).to(device)

    expo = net(t_frame)
    print(expo)
    
    cv2.imshow('original', oframe)
   
    cv2.waitKey(33)

cap.release()    
cv2.destroyAllWindows()
