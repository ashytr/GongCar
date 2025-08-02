import cv2
import numpy as np

def nothing(x):
    pass

# 创建窗口和滑动条
cv2.namedWindow('Line Detection')
cv2.createTrackbar('Canny1', 'Line Detection', 50, 255, nothing)
cv2.createTrackbar('Canny2', 'Line Detection', 150, 255, nothing)
cv2.createTrackbar('HoughLines Threshold', 'Line Detection', 100, 200, nothing)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    # 获取当前滑动条的值
    threshold1 = cv2.getTrackbarPos('Canny1', 'Line Detection')
    threshold2 = cv2.getTrackbarPos('Canny2', 'Line Detection')
    hough_threshold = cv2.getTrackbarPos('HoughLines Threshold', 'Line Detection')
    
    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 应用高斯模糊降噪
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 使用Canny边缘检测
    edges = cv2.Canny(blurred, threshold1, threshold2)
    
    # 霍夫直线检测
    vis_frame = frame.copy()
    lines = cv2.HoughLines(edges, 1, np.pi/180, hough_threshold)
    
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
            
            # 计算角度
            dy = -(y2 - y1)
            dx = x2 - x1
            angle = np.arctan2(dy, dx) * 180.0 / np.pi
            
            # 确保角度在-90到90度之间
            if angle > 90:
                angle = angle - 180
            elif angle < -90:
                angle = angle + 180
                
            # 绘制检测到的线和角度信息
            cv2.line(vis_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(vis_frame, f"Angle: {angle:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # 显示结果
    cv2.imshow('Line Detection', vis_frame)
    cv2.imshow('Edges', edges)
    cv2.imshow('Gray', gray)
    
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
