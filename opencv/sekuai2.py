import cv2
import numpy as np
import math
import serial
import struct
from collections import  deque
cap = cv2.VideoCapture(0)
cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(3,640)
cap.set(4,480)
cap2=cv2.VideoCapture(2)
qrDecoder= cv2.QRCodeDetector()
cap2.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
# 初始化 QRCode 检测器
greenLower= np.array([0,107,146]) #设定阈值，HSV空间 
greenUpper = np.array([179,255,255])
mybuffer = 64  #初始化追踪点的列表
pts = deque(maxlen=mybuffer) 
'''
ser = serial.Serial("/dev/ttyAMA0",115200)
if ser.isOpen == False:
    ser.open()
    
def send_data_packet(x, y):
    temp = struct.pack("<bbhhhhb",  # 格式为俩个字符俩个整型
                       0x2C,  # 帧头1
                       0x3C,  # 帧头2
                       int(x),  # up sample by 4    #数据1
                       int(y),  # up sample by 4    #数据2
                       int(2),
                       int(3),
                       0x5b)
    ser.write(temp)  # 串口发送
'''
    


while (True):
    # 从摄像头读取图像
    ret,img2 = cap2.read()
    if not ret:
        print("No Camera")
    # 对图像进行QR码检测和解码
    data, bbox, straight_qrcode = qrDecoder.detectAndDecode(img2)
    
    # 如果检测到QR码且边界框数据有效
    if bbox is not None and len(bbox) > 0 and len(bbox[0]) == 4:  
        points = bbox[0]  # 获取QR码的四个角点坐标
        # 计算左上角和右下角点的坐标
        top_left = (int(points[0][0]), int(points[0][1]))
        bottom_right = (int(points[2][0]), int(points[2][1]))
        
        # 在图像上绘制QR码的边界框(绿色)
        cv2.rectangle(img2, top_left, bottom_right, (0, 255, 0), 2)
        
        # 设置文本显示位置和参数
        text_position = (top_left[0], top_left[1] - 10)  # 文本位置在边界框上方
        font = cv2.FONT_HERSHEY_SIMPLEX  # 设置字体
        font_scale = 0.5  # 字体大小
        font_color = (0, 0, 255)  # 字体颜色(红色)
        thickness = 1  # 字体粗细
        
        # 在图像上显示解码得到的QR码内容
        cv2.putText(img2, data, text_position, font, font_scale, font_color, thickness)
        
    cv2.imshow("Image with QR Code", img2)
    cv2.waitKey(1)

    ret, frame = cap.read()
    #rows, cols = frame.shape[:2]
    #M = cv2.getRotationMatrix2D((cols / 2, rows / 2), 20, 1)
    #第一个参数旋转中心，第二个参数旋转角度，第三个参数：缩放比例
    #自适应图片边框大小
    '''
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    new_w = rows * sin + cols * cos
    new_h = rows * cos + cols * sin
    M[0, 2] += (new_w - cols) * 0.5
    M[1, 2] += (new_h - rows) * 0.5
    w = int(np.round(new_w))
    h = int(np.round(new_h))
    '''
    #new frame
    #frame = cv2.warpAffine(frame, M, (cols, rows))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=0)
    mask= cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts_list = []
    cnts_list_1 = []
    if len(cnts) > 0:
        for i in range(len(cnts)):
            area = cv2.contourArea(cnts[i])
            if area > 2000 and area <= 60000:
                cnts_list.append(cnts[i])
        if len(cnts_list)>0:
            for i in range(len(cnts_list)):
                rect_green = cv2.minAreaRect(cnts_list[i])
                if rect_green[1][0]/rect_green[1][1] <= 1.1 and rect_green[1][0]/rect_green[1][1] >= 0.9:
                    cnts_list_1.append(cnts_list[i])
            if len(cnts_list_1) > 0:       
                c_green = max(cnts_list_1, key = cv2.contourArea)
                #epsilon = 0.1 * cv2.arcLength(c_green, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                #approx = cv2.approxPolyDP(c_green, epsilon, True)
                corners = len(c_green)
                
                #if corners >= 4 and corners <= 10:
                rect_green = cv2.minAreaRect(c_green)  #确定面积最大的轮廓的juxing
                box_green = cv2.boxPoints(rect_green)
                max_green = rect_green[1][0]*rect_green[1][1]
                cv2.drawContours(frame,[np.int0(c_green)] ,-1, (0, 0, 255), 2)  #计算质心
                cv2.drawContours(frame, [np.int0(box_green)],-1, (0, 255, 255), 2)
                x = rect_green[0][0] 
                y = rect_green[0][1] 
            
                #print(box_green)
                #print('area:' + str(max_green))
                #send_data_packet(x,y)
   
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break
print(data)
print(int(x))
print(int(y))
cap.release()
cv2.destroyAllWindows()

