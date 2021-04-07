#!/usr/bin/env python

import cv2,glob,os
 
 
vedio_files=glob.glob("*.mkv")
caps=[cv2.VideoCapture(name) for name in vedio_files]

#cap_length=max([int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) for cap in caps])


if not os.path.isdir("./image/"):
    os.mkdir("./image/")

#for i in range(abs(cap_length)):

for cap in caps:
    i=0
    while True:
    	cap_name=str(cap).split(" ")[1][:-1]
    	ret, img = cap.read()
    	
    	if not ret:
       	   break
      
        img=cv2.resize(img, dsize=(200,112))
        
        img=img[46:,:]
        
        cv2.imwrite("image/"+str(i)+"-"+cap_name+".jpg",img) 
 
        i+=1

for cap, vedio_file in zip(caps, vedio_files):
    current_name=str(vedio_file).split(".")[0]
    cap_name=str(cap).split(" ")[1][:-1]
    os.rename(current_name+".mkv", cap_name+".mkv")
    os.rename(current_name+".csv", cap_name+".csv")


for cap in caps:
    cap.release()

cv2.destroyAllWindows()


