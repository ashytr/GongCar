import cv2
import numpy as np
import math
import time
cap=cv2.VideoCapture(0)
#cap.set(3,1080)
#cap.set(4,1080)
cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
kernel = np.ones((5, 5), np.uint8)
while (True):
    xy=None
    redLower_1= np.array([0, 120, 64]) #设定红色阈值，HSV空间 
    redUpper_1 = np.array([4, 255, 234])
    redLower= np.array([151, 67, 0]) #设定红色阈值，HSV空间 
    redUpper = np.array([179, 255, 234])
    blueLower = np.array([86, 124, 144])
    blueUpper = np.array([125, 255, 255])
    greenLower= np.array([57,53,54]) #设定阈值，HSV空间 
    greenUpper = np.array([90,255,255])
    mybuffer = 64  #初始化追踪点的列表
    ret, frame = cap.read()
    if not ret:
        print ('No Camera')
        break
    #frame = frame[0:300,0:300]
    max_red = 0
    max_blue = 0
    max_green = 0
    max_red_1 = 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   #转到HSV空间   
    mask1 = cv2.inRange(hsv, redLower, redUpper)  #根据阈值构建掩膜
    mask2 = cv2.inRange(hsv, blueLower, blueUpper)
    mask3 = cv2.inRange(hsv, greenLower, greenUpper)
    mask4 = cv2.inRange(hsv, redLower_1, redUpper_1)
    mask1 = cv2.erode(mask1, kernel, iterations=2)  #腐蚀操作
    mask2 = cv2.erode(mask2, kernel, iterations=2)
    mask3 = cv2.erode(mask3,kernel, iterations=0)
    mask4 = cv2.erode(mask4, kernel, iterations=2)
    mask1 = cv2.dilate(mask1, kernel, iterations=2)#膨胀操作，其实先腐蚀再膨胀的效果是开运算，去除噪点
    mask2 = cv2.dilate(mask2, kernel, iterations=2)
    mask3 = cv2.dilate(mask3,kernel, iterations=5)
    mask4 = cv2.dilate(mask4, kernel, iterations=2)
    cnts1= cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  #轮廓检测
    cnts2= cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts3= cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts4= cv2.findContours(mask4.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None  #初始化瓶盖圆形轮廓质心
        #如果存在轮廓  
    #time.sleep(1)
    
    if len(cnts1) > 0:    
        c_red = max(cnts1, key = cv2.contourArea) #找到面积最大的轮廓   
        rect_red = cv2.minAreaRect(c_red)  #确定面积最大的轮廓的juxing
        max_red = rect_red[1][0]*rect_red[1][1]
        box_red = cv2.boxPoints(rect_red)  
        #print(rect_red)
        #cv2.drawContours(frame, [np.int0(box_red)],-1, (0, 255, 255), 2)  #计算质心
        #print('red')
        #print(c_red)
       
    if len(cnts2) > 0:    
        c_blue = max(cnts2, key = cv2.contourArea)  
        rect_blue = cv2.minAreaRect(c_blue)  #确定面积最大的轮廓的juxing
        max_blue = rect_blue[1][0]*rect_blue[1][1]
        box_blue = cv2.boxPoints(rect_blue)  
        #cv2.drawContours(frame, [np.int0(box_blue)],-1, (0, 255, 255), 2)  #计算质心
        #print('blue')
        #print(c_blue)
        
    if len(cnts3) > 0:
        c_green = max(cnts3, key = cv2.contourArea)  
        rect_green = cv2.minAreaRect(c_green)  #确定面积最大的轮廓的juxing
        max_green = rect_green[1][0]*rect_green[1][1]
        box_green = cv2.boxPoints(rect_green)  
        x,y,angle=cv2.minAreaRect(c_green)
        #print(x)
        if 310<=x[0]<=320 and 230<=x[1]<=240:
            print(1)
    #cv2.drawContours(frame, [np.int0(box_green)],-1, (0, 255, 255), 2)  #计算质心
      
    if len(cnts4) > 0:    
        c_red_1 = max(cnts4, key = cv2.contourArea) #找到面积最大的轮廓   
        rect_red_1 = cv2.minAreaRect(c_red_1)  #确定面积最大的轮廓的juxing
        max_red_1 = rect_red_1[1][0]*rect_red_1[1][1]
        box_red_1 = cv2.boxPoints(rect_red_1)
    
    if (max_red > max_green and max_red > max_blue and max_red > 1000):
        cv2.drawContours(frame, [np.int0(box_red)],-1, (0, 0, 255), 2)
        #print(rect_red)
     
        #send_data_packet(rect_red[0][0],1)
    elif max_green > max_red and max_green > max_blue and max_green > 1000:
        cv2.drawContours(frame, [np.int0(box_green)],-1, (0, 255, 0), 2)
        #print(rect_green)
        #send_data_packet(rect_green[0][0],2)
    elif max_blue > max_red and max_blue > max_green and max_blue > 1000:
        cv2.drawContours(frame, [np.int0(box_blue)],-1, (255, 0, 0), 2)
        #print(rect_blue)
        #send_data_packet(rect_blue[0][0],3)
    elif (max_red_1 > max_green and max_red_1 > max_blue and max_red_1 > max_red and max_red_1 > 1000):
        cv2.drawContours(frame, [np.int0(box_red_1)],-1, (0, 0, 255), 2)
        #print(rect_red)
        #send_data_packet(rect_red_1[0][0],1)
    cv2.imshow('Frame', mask3)
            