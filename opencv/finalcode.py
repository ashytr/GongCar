# 导入所需的库
import cv2  # OpenCV库，用于图像处理
import numpy as np  # 数值计算库
import struct  # 处理字节数据的库
import serial  # 串口通信库
import time  # 时间处理库

# # 初始化第一个摄像头(二维码识别摄像头)
# cap=cv2.VideoCapture(0)  # 打开摄像头0
# cap.set(3,650)  # 设置画面宽度
# cap.set(4,650)  # 设置画面高度
# cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))  # 设置视频编码格式为MJPG
 
# 初始化第二个摄像头(颜色识别摄像头)
cap2=cv2.VideoCapture(2)  # 打开摄像头2
cap2.set(3,650)  # 设置画面宽度
cap2.set(4,650)  # 设置画面高度
cap2.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))  # 设置视频编码格式为MJPG

# 创建形态学操作的核
kernel=np.ones((5, 5), np.uint8)  # 5x5的矩形核，用于图像膨胀和腐蚀操作

# 初始化串口通信
ser= serial.Serial("/dev/ttyAMA3",115200)  # 打开串口，波特率115200

if ser.isOpen == False:
    ser.open()            

    
def send_data_packet(mode,color,cx,cy,o,angle,catch):
    # 创建要发送的数据，首先是两个字节数据组成的 bytearray，然后分别将三个字符串编码后连接起来
    data_to_send =bytes([0xFF,0x22,mode, color,cx//100,(cx//10)%10,cx%10,cy//100,(cy//10)%10,cy%10,o,angle//10,angle%10,catch])
    # 通过串口发送数据
    print(data_to_send)
    ser.write(data_to_send)
    #print(data_to_send)    
#binary    
'''
def send_data_packet(mode,color,cx,cy,angle,catch):
    temp = struct.pack("<bbbbHHHbb",  
                       0x99,  # 帧头1
                       0x2c,  # 帧头2
                       mode,
                       color,
                       int(cx),  
                       int(cy),  
                       int(-angle),
                       catch,
                       0x5b)
    ser.write(temp)  # 串口发送
'''
def flag():
    alldata=ser.read(14)
    re=struct.unpack("bbbbbbbbbbbbbb",alldata)
    #print(re)
    if re[0]==-1 and re[1]==34:
        return re[2]
    elif re[1]==-1 and re[2]==34:
        return re[3]
    elif re[2]==-1 and re[3]==34:
        return re[4]
    elif re[3]==-1 and re[4]==34:
        return re[5]
    elif re[4]==-1 and re[5]==34:
        return re[6]
    elif re[5]==-1 and re[6]==34:
        return re[7]
    elif re[6]==-1 and re[7]==34:
        return re[8]
    elif re[7]==-1 and re[8]==34:
        return re[9]
    elif re[8]==-1 and re[9]==34:
        return re[10]
    elif re[9]==-1 and re[10]==34:
        return re[11]
    elif re[10]==-1 and re[11]==34:
        return re[12]
    elif re[11]==-1 and re[12]==34:
        return re[13]
    
'''
def flag():
    alldata=ser.read(14)
    re=struct.unpack("<bbbbbbbbbbbbbb",alldata)
    if re[0]==-1 and re[1]==34:
        return re[2]
'''
    
# def qrcode(): 
#     qrDecoder = cv2.QRCodeDetector()
#     a=1
#     while (a):
#         ret,img = cap.read()
#         if not ret:
#             #print("No Camera")
#             continue
#         data, bbox, straight_qrcode = qrDecoder.detectAndDecode(img)
#         if len(data)>0:
#             a=0
#             print(data)
#     return data
# '''#QRCODESHOW
#         if len(data>0):    
#         #print(bbox[0])
#         if bbox is not None and len(bbox) > 0 and len(bbox[0]) == 4:  
#             points = bbox[0]
#             top_left = (int(points[0][0]), int(points[0][1]))
#             bottom_right = (int(points[2][0]), int(points[2][1]))
#             #print(top_left)
#             #print(bottom_right)
#             cv2.rectangle(img, top_left, bottom_right, (0, 255, 0), 2)
#             text_position = (top_left[0], top_left[1] - 10)
#             font = cv2.FONT_HERSHEY_SIMPLEX
#             font_scale = 0.5
#             font_color = (0, 0, 255) 
#             thickness = 1
#     cv2.putText(img, data, text_position, font, font_scale, font_color, thickness)
#     cv2.imshow("Image with QR Code", img)
#     cv2.waitKey(1)
#         '''
def road():
    while True:
        while True:
            ret,frame = cap2.read()
            if not ret:
                #print ('No Camera')
                continue
            else:
                break          
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 100)
        if lines is not None:
            for rho, theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            if(angle<0):
                send_data_packet(0,0,0,0,0,int(-angle+1),0)
                break
            else:
                send_data_packet(0,0,0,0,0,int(90-angle+1),0)
                break
            
        #time.sleep(0.01)
        
        
        
        
def red():
    while True:
        redLower = np.array([0,142,97])
        redUpper = np.array([13,255,255])
        while True:
            ret, frame = cap2.read()
            if not ret:
                #print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
                    rect_red = cv2.minAreaRect(cnts_list[i])
                    if rect_red[1][0]/rect_red[1][1] <= 1.1 and rect_red[1][0]/rect_red[1][1] >= 0.9:
                        cnts_list_1.append(cnts_list[i])
                if len(cnts_list_1) > 0:       
                    c_red = max(cnts_list_1, key = cv2.contourArea)
                    epsilon = 0.1 * cv2.arcLength(c_red, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                    approx = cv2.approxPolyDP(c_red, epsilon, True)
                    corners = len(c_red)
                    #if corners >= 4 and corners <= 10:
                    rect_red = cv2.minAreaRect(c_red)  #确定面积最大的轮廓的juxing
                    box_red = cv2.boxPoints(rect_red)
                    max_red = rect_red[1][0]*rect_red[1][1]
                    x = rect_red[0][0] 
                    y = rect_red[0][1]
                    send_data_packet(0,0,int(x),int(y),0,0,0)
                    break

    
    
    
    
def red_color():
    while True:
        redLower = np.array([0,113,147])
        redUpper = np.array([179,255,255])
        while True:
            ret, frame = cap2.read()
            if not ret:
                #print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
                    rect_red = cv2.minAreaRect(cnts_list[i])
                    if rect_red[1][0]/rect_red[1][1] <= 1.1 and rect_red[1][0]/rect_red[1][1] >= 0.9:
                        cnts_list_1.append(cnts_list[i])
                if len(cnts_list_1) > 0:       
                    c_red = max(cnts_list_1, key = cv2.contourArea)
                    epsilon = 0.1 * cv2.arcLength(c_red, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                    approx = cv2.approxPolyDP(c_red, epsilon, True)
                    corners = len(c_red)
                    #if corners >= 4 and corners <= 10:
                    rect_red = cv2.minAreaRect(c_red)  #确定面积最大的轮廓的juxing
                    box_red = cv2.boxPoints(rect_red)
                    max_red = rect_red[1][0]*rect_red[1][1]
                    x = rect_red[0][0] 
                    y = rect_red[0][1]
                    send_data_packet(0,0,int(x),int(y),0,0,0)
                    break

    
        

def green():
    while True:
        greenLower = np.array([36,87,168])
        greenUpper = np.array([179,255,255])
        while True:
            ret, frame = cap2.read()
            if not ret:
                #print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
                    epsilon = 0.1 * cv2.arcLength(c_green, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                    approx = cv2.approxPolyDP(c_green, epsilon, True)
                    corners = len(c_green)

                    #if corners >= 4 and corners <= 10:
                    rect_green = cv2.minAreaRect(c_green)  #确定面积最大的轮廓的juxing
                    box_green = cv2.boxPoints(rect_green)
                    max_green = rect_green[1][0]*rect_green[1][1]
                    x = rect_green[0][0] 
                    y = rect_green[0][1]
                    send_data_packet(0,0,int(x),int(y),0,0,0)
                    break
    
        
def green_color():
    while True:
        greenLower = np.array([31,90,114])
        greenUpper = np.array([179,255,255])
        while True:
            ret, frame = cap2.read()
            if not ret:
                print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
                    epsilon = 0.1 * cv2.arcLength(c_green, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                    approx = cv2.approxPolyDP(c_green, epsilon, True)
                    corners = len(c_green)
                    #if corners >= 4 and corners <= 10:
                    rect_green = cv2.minAreaRect(c_green)  #确定面积最大的轮廓的juxing
                    box_green = cv2.boxPoints(rect_green)
                    max_green = rect_green[1][0]*rect_green[1][1]
                    x = rect_green[0][0] 
                    y = rect_green[0][1]
                    send_data_packet(0,0,int(x),int(y),0,0,0)
                    break
    
                
def blue():
    while True:
        blueLower = np.array([60,142,154])
        blueUpper = np.array([142,255,255])
        while True:
            ret, frame = cap2.read()
            if not ret:
                #print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, blueLower, blueUpper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
                    rect_blue = cv2.minAreaRect(cnts_list[i])
                    if rect_blue[1][0]/rect_blue[1][1] <= 1.1 and rect_blue[1][0]/rect_blue[1][1] >= 0.9:
                        cnts_list_1.append(cnts_list[i])
                if len(cnts_list_1) > 0:       
                    c_blue = max(cnts_list_1, key = cv2.contourArea)
                    epsilon = 0.1 * cv2.arcLength(c_blue, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                    approx = cv2.approxPolyDP(c_blue, epsilon, True)
                    corners = len(c_blue)

                    #if corners >= 4 and corners <= 10:
                    rect_blue = cv2.minAreaRect(c_blue)  
                    box_blue = cv2.boxPoints(rect_blue)
                    max_blue = rect_blue[1][0]*rect_blue[1][1]
                    x = rect_blue[0][0] 
                    y = rect_blue[0][1]
                    send_data_packet(0,0,int(x),int(y),0,0,0)
                    break

    
    
    
    
def blue_color():
    
    while True:
        blueLower = np.array([91,30,52])
        blueUpper = np.array([179,255,154])
        while True:
            ret, frame = cap2.read()
            if not ret:
                #print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, blueLower, blueUpper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
                    rect_blue = cv2.minAreaRect(cnts_list[i])
                    if rect_blue[1][0]/rect_blue[1][1] <= 1.1 and rect_blue[1][0]/rect_blue[1][1] >= 0.9:
                        cnts_list_1.append(cnts_list[i])
                if len(cnts_list_1) > 0:       
                    c_blue = max(cnts_list_1, key = cv2.contourArea)
                    epsilon = 0.1 * cv2.arcLength(c_blue, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                    approx = cv2.approxPolyDP(c_blue, epsilon, True)
                    corners = len(c_blue)

                    #if corners >= 4 and corners <= 10:
                    rect_blue = cv2.minAreaRect(c_blue)  
                    box_blue = cv2.boxPoints(rect_blue)
                    max_blue = rect_blue[1][0]*rect_blue[1][1]
                    x = rect_blue[0][0] 
                    y = rect_blue[0][1]
                    send_data_packet(0,0,int(x),int(y),0,0,0)
                    break

    

def center():
    while (True):
        whiteLower= np.array([0, 0, 0]) #设定阈值，HSV空间 
        whiteUpper = np.array([180, 255, 255])
        while True:
            ret, frame = cap2.read()
            if not ret:
            #print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   #转到HSV空间   
        mask5= cv2.inRange(hsv, whiteLower, whiteUpper)  #根据阈值构建掩膜
        mask5= cv2.erode(mask5, kernel, iterations=2)
        mask5= cv2.dilate(mask5, kernel, iterations=2)
        cnts5= cv2.findContours(mask5.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts5) > 0:    
            c_white= max(cnts5, key = cv2.contourArea) #找到面积最大的轮廓   
            rect_white = cv2.minAreaRect(c_white)  #确定面积最大的轮廓
            max_white = rect_white[1][0]*rect_white[1][1]
            box_white = cv2.boxPoints(rect_white)
            #print(rect_red)
            xy,wh,angle=rect_white = cv2.minAreaRect(c_white)
            send_data_packet(0,0,int(xy[0]),int(xy[1]),0,0,0)
            break
        
def redpoint():
    while (True):
        redLower = np.array([0,142,97])
        redUpper = np.array([13,255,255])
        while True:
            ret, frame = cap2.read()
            if not ret:
            #print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
                    rect_red = cv2.minAreaRect(cnts_list[i])
                    if rect_red[1][0]/rect_red[1][1] <= 1.1 and rect_red[1][0]/rect_red[1][1] >= 0.9:
                        cnts_list_1.append(cnts_list[i])
                if len(cnts_list_1) > 0:       
                    c_red = max(cnts_list_1, key = cv2.contourArea)
                    epsilon = 0.1 * cv2.arcLength(c_red, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                    approx = cv2.approxPolyDP(c_red, epsilon, True)
                    corners = len(c_red)
                #if corners >= 4 and corners <= 10:
                    rect_red = cv2.minAreaRect(c_red)  #确定面积最大的轮廓的juxing
                    box_red = cv2.boxPoints(rect_red)
                    max_red = rect_red[1][0]*rect_red[1][1]
                    x = rect_red[0][0] 
                    y = rect_red[0][1]
                    if 200<=x<=420 and 200<=y<=420 :     #误差在5左右可以识别，可以再进行微调
                        send_data_packet(0,1,0,0,0,0,1)
                        break

def greenpoint():
    while(True):
        greenLower = np.array([55,106,94])
        greenUpper = np.array([179,255,255])
        while True:
            
            ret, frame = cap2.read()
            if not ret:
                #print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
                    epsilon = 0.1 * cv2.arcLength(c_green, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                    approx = cv2.approxPolyDP(c_green, epsilon, True)
                    corners = len(c_green)
                #if corners >= 4 and corners <= 10:
                    rect_green = cv2.minAreaRect(c_green)  #确定面积最大的轮廓的juxing
                    box_green = cv2.boxPoints(rect_green)
                    max_green = rect_green[1][0]*rect_green[1][1]
                    x = rect_green[0][0] 
                    y = rect_green[0][1]
                    if (200<=x<=420  and  200<=y<=420) : 
                        send_data_packet(0,2,0,0,0,0,1)
                        #print(1)
                        break
    
    
    
def bluepoint():
    while(True):
        blueLower = np.array([60,142,154])
        blueUpper = np.array([142,255,255])
        while True:
            ret, frame = cap2.read()
            if not ret:
                #print ('No Camera')
                continue
            else:
                break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, blueLower, blueUpper)
        mask = cv2.erode(mask, kernel, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
                    rect_blue = cv2.minAreaRect(cnts_list[i])
                    if rect_blue[1][0]/rect_blue[1][1] <= 1.1 and rect_blue[1][0]/rect_blue[1][1] >= 0.9:
                        cnts_list_1.append(cnts_list[i])
                if len(cnts_list_1) > 0:       
                    c_blue = max(cnts_list_1, key = cv2.contourArea)
                    epsilon = 0.1 * cv2.arcLength(c_blue, True) #多边形拟合的距离参数，下一个函数要用到。原理见代码后链接
                    approx = cv2.approxPolyDP(c_blue, epsilon, True)
                    corners = len(c_blue)

                    #if corners >= 4 and corners <= 10:
                    rect_blue = cv2.minAreaRect(c_blue)  
                    box_blue = cv2.boxPoints(rect_blue)
                    max_blue = rect_blue[1][0]*rect_blue[1][1]
                    x = rect_blue[0][0] 
                    y = rect_blue[0][1]
                    if 2<=x<=1420  and  2<=y<=1400 : 
                        send_data_packet(0,3,0,0,0,0,1)
                        break
i=0
b=1
c=1
d=1
data=[0,0,0,0,0,0,0]

while(True):
    
    task=flag()
    
    
    if (task==1):
         print(1)
    #     data=qrcode()
        
       
        
        
        
    # elif (task==2):
    #     print(2)
    #     #print(data)
    #     if (data[0+i]=='1'and data[1+i]=='2'and data[2+i]=='3'):
    #         bluepoint()
    #         time.sleep(5)
    #         redpoint()
    #         time.sleep(5)
    #         greenpoint()
            
    #     elif(data[0+i]=='1' and data[1+i]=='3'and data[2+i]=='2'):
    #         i=i+4
    #         redpoint()
    #         time.sleep(5)
    #         bluepoint()
    #         time.sleep(5)
    #         greenpoint()
    #     elif(data[0+i]=='2'and data[1+i]=='1'and data[2+i]=='3'):
    #         i=i+4
    #         greenpoint()
    #         time.sleep(5)
    #         redpoint() 
    #         time.sleep(5)
    #         bluepoint()
    #     elif(data[0]=='2'and data[1+i]=='3'and data[2+i]=='1'):
    #         i=i+4
    #         greenpoint()
    #         time.sleep(5)
    #         bluepoint()
    #         time.sleep(5)
    #         redpoint()
    #     elif(data[0+i]=='3'and data[1+i]=='1'and data[2+i]=='2'):
    #         i=i+4
    #         bluepoint()
    #         time.sleep(5)
    #         redpoint() 
    #         time.sleep(5)
    #         greenpoint()
    #     elif(data[0+i]=='3'and data[1+i]=='2'and data[2+i]=='1'):
    #         i=i+4
    #         bluepoint()
    #         time.sleep(5)
    #         greenpoint()
    #         time.sleep(5)
    #         redpoint() 
    #     else:
    #         print("no")
    
    elif(task==3):
        
        print(3)
        center()
    
    elif (task==4 and b==1):
        
        print(4)
        red_color()
        b=0
        
    
    elif (task==5 and c==1):
        print(5)
        green_color()
        c=0
        time.sleep(5)
    
    elif (task==6 and d==1):
        print(6)
        blue_color()
        d=0
        
    
    elif (task==7):
        print(7)
        road()   
            
            
    elif (task==8):
        print(8)
        red()
        
        
    elif (task==9):
        print(9)
        green()
        
    elif (task==10):
        print(10)
        blue()
    elif(task==11):
        print(11)
        bluepoint()
        
        
        
    else:
        pass

















