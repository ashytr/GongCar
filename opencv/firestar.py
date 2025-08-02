import cv2
import numpy as np
import time
import serial
import struct

ser= serial.Serial('/dev/ttyAMA3', 115200)  # 根据实际串口设备修改端口
# 打开摄像头
cap = cv2.VideoCapture(0)

if ser.isOpen == False:
    ser.open()            
    
def send_data_packet(counts):
    data_to_send =bytes([0xeb,0x90,counts,0x5b])
    # 通过串口发送数据
    print(data_to_send)
    ser.write(data_to_send)

    
def flag():
    alldata=ser.read(4)
    re=struct.unpack("bbbb",alldata)
    print(re)
    if re[0]==-21 and re[1]==-112 and re[3]==91:
        return re[2]





def roi_brightness_detection():
    # 设置摄像头分辨率（可根据需要调整）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_EXPOSURE, 50)
    # 初始化计数和标志位
    count = 0
    flag = 1
    # 用于存储明度阈值
    threshold_value = 193

    # 记录上一次的count值
    prev_count = count
   
    no_change_threshold = 8
    # 记录开始时间
    start_time = time.time()

    while True:
        # 从摄像头读取一帧图像
        ret, frame = cap.read()

        if ret:
            # 将图像转换为HSV颜色空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 这里将感兴趣区域往左和上移一些，例如各移动30个像素，可根据实际需求调整移动距离
            roi = hsv[60:160, 330:390]

            # 计算ROI的平均明度值
            mean_val = cv2.mean(roi)[2]

            # 根据当前设置的阈值进行检测
            if mean_val < threshold_value and mean_val > 0 and flag == 1:
                count += 1
                flag = 0
            if mean_val > threshold_value:
                flag = 1

            # 检查count是否长时间未变化
            if count == prev_count:
                current_time = time.time()
                if current_time - start_time > no_change_threshold:
                    send_data_packet(count)
                    break
            else:
                prev_count = count
                start_time = time.time()

        else:
            break


    
     




def transportatiaon():
    
    cap.set(3, 640)
    cap.set(4, 480)
    cap.set(cv2.CAP_PROP_EXPOSURE, 50)

    kernel = np.ones((15, 15), np.uint8)

    # 绿色的HSV范围
    greenLower = np.array([66, 165, 152])
    greenUpper = np.array([110, 255, 255])

    # 红色的HSV范围
    redLower = np.array([0, 0,225])
    redUpper = np.array([74, 38, 255])

    while True:
        ret, frame = cap.read()
        if not ret:
            print("no camera")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 处理绿色色块检测
        green_mask = cv2.inRange(hsv, greenLower, greenUpper)
        green_mask = cv2.erode(green_mask, kernel, iterations=0)
        green_mask = cv2.dilate(green_mask, kernel, iterations=1)
        green_cnts = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        max_green_area = 0

        if len(green_cnts) > 0:
            for cnt in green_cnts:
                area = cv2.contourArea(cnt)
                if area > max_green_area:
                    max_green_area = area
            print(max_green_area)

        if max_green_area > 700 :
            print(1)
            send_data_packet(2)
            break
        else:
            print(2)
            send_data_packet(1)
            time.sleep(0.5)
            break
        ''' 
        # 处理红色色块检测
        red_mask = cv2.inRange(hsv, redLower, redUpper)
        red_mask = cv2.erode(red_mask, kernel, iterations=0)
        red_mask = cv2.dilate(red_mask, kernel, iterations=1)
        red_cnts = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        max_red_area = 0

        if len(red_cnts) > 0:
            for cnt in red_cnts:
                area = cv2.contourArea(cnt)
                if area > max_red_area:
                    max_red_area = area
            print(max_red_area)

        if max_red_area > 200:
            print(0)
            send_data_packet(1)
        '''
            

            
def kuku():
   # 设置摄像头分辨率（
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # 用于存储明度阈值
    threshold_value =209 
    while True:
        # 从摄像头读取一帧图像
        ret, frame = cap.read()
        if ret:
            # 将图像转换为HSV颜色空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                # 调换感兴趣区域的长宽，原来的宽变成高，高变成宽
            roi = hsv[90:150, 360:380]               
                # 计算ROI的平均明度值
            mean_val = cv2.mean(roi)[2]  
            if mean_val < threshold_value and mean_val > 0:         
                send_data_packet(4)
                break
                         
            
            
            
            
            
            
            
            
            
            
            
            
def ku():  
    cap.set(3, 640)
    cap.set(4, 480)
    kernel = np.ones((15, 15), np.uint8)
    blueLower = np.array([84, 101, 38])
    blueUpper = np.array([132, 255, 176])
    # 根据原多边形坐标大致确定矩形的坐标
    x1 = min(175, 207, 235, 275)
    y1 = min(34, 181, 42, 166)
    x2 = max(175, 207, 235, 275)
    y2 = max(34, 181, 42, 166)
    # 定义矩形感兴趣区域的坐标
    roi_rect_pts = np.array([[(x1, y1), (x2, y1), (x2, y2), (x1, y2)]], dtype=np.int32)


    while True:
        ret, frame = cap.read()
        if not ret:
            print("no camera")
            break

        # 绘制矩形感兴趣区域
        cv2.polylines(frame, roi_rect_pts, True, (0, 255, 0), 2)

        # 获取矩形感兴趣区域的图像
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, roi_rect_pts, 255)
        roi_frame = cv2.bitwise_and(frame, frame, mask=mask)

        hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, blueLower, blueUpper)
        blue_mask = cv2.erode(blue_mask, kernel, iterations=0)
        blue_mask = cv2.dilate(blue_mask, kernel, iterations=0)
        blue_cnts = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        max_blue_area = 0

        if len(blue_cnts) > 0:
            for cnt in blue_cnts:
                area = cv2.contourArea(cnt)
                if area > max_blue_area:
                    max_blue_area = area
            print(max_blue_area)

        if max_blue_area > 200:
            print(1)
            # 假设这里的send_data_packet函数已定义并可正确调用
            send_data_packet(3)
            break
    
            


if __name__ == "__main__":
    while(True):
    
        task=flag()
        if(task==1):
            roi_brightness_detection()
        if(task==2):
            transportatiaon()
        if(task==3):
            kuku()
        


        

         



