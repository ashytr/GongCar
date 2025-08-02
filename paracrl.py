import cv2
import numpy as np
import json
import os
import time  # 添加time模块导入

def create_hsv_trackbars():
    def nothing(x):
        pass
    
    # 创建控制窗口
    cv2.namedWindow('HSV Controls', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Ring HSV Controls', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Line Controls', cv2.WINDOW_NORMAL)  # 改为Line Controls
    cv2.namedWindow('LAB Controls', cv2.WINDOW_NORMAL)  # 新增LAB控制窗口
    
    # 为三种颜色分别创建 HSV 滑动条
    colors = ['red', 'green', 'blue']
    for color in colors:
        cv2.createTrackbar(f'C_{color}_H1', 'HSV Controls', 0, 179, nothing)
        cv2.createTrackbar(f'C_{color}_H2', 'HSV Controls', 179, 179, nothing)
        cv2.createTrackbar(f'C_{color}_S1', 'HSV Controls', 0, 255, nothing)
        cv2.createTrackbar(f'C_{color}_S2', 'HSV Controls', 255, 255, nothing)
        cv2.createTrackbar(f'C_{color}_V1', 'HSV Controls', 0, 255, nothing)
        cv2.createTrackbar(f'C_{color}_V2', 'HSV Controls', 255, 255, nothing)
    
    # 直线检测参数控制
    cv2.createTrackbar('Canny1', 'Line Controls', 50, 255, nothing)
    cv2.createTrackbar('Canny2', 'Line Controls', 150, 255, nothing)
    cv2.createTrackbar('HoughLines Threshold', 'Line Controls', 100, 200, nothing)
    
    # 为圆环添加独立的HSV控制条
    for color in colors:
        cv2.createTrackbar(f'R_{color}_H1', 'Ring HSV Controls', 0, 179, nothing)
        cv2.createTrackbar(f'R_{color}_H2', 'Ring HSV Controls', 179, 179, nothing)
        cv2.createTrackbar(f'R_{color}_S1', 'Ring HSV Controls', 0, 255, nothing)
        cv2.createTrackbar(f'R_{color}_S2', 'Ring HSV Controls', 255, 255, nothing)
        cv2.createTrackbar(f'R_{color}_V1', 'Ring HSV Controls', 0, 255, nothing)
        cv2.createTrackbar(f'R_{color}_V2', 'Ring HSV Controls', 255, 255, nothing)
    
    # 添加LAB空间控制滑动条（灰色）
    cv2.createTrackbar('Gray_L_Min', 'LAB Controls', 128, 255, nothing)
    cv2.createTrackbar('Gray_L_Max', 'LAB Controls', 181, 255, nothing)
    cv2.createTrackbar('Gray_A_Min', 'LAB Controls', 119, 255, nothing)
    cv2.createTrackbar('Gray_A_Max', 'LAB Controls', 136, 255, nothing)
    cv2.createTrackbar('Gray_B_Min', 'LAB Controls', 113, 255, nothing)
    cv2.createTrackbar('Gray_B_Max', 'LAB Controls', 131, 255, nothing)

def get_hsv_values(color):
    """获取物块颜色的HSV阈值"""
    h_min = cv2.getTrackbarPos(f'C_{color}_H1', 'HSV Controls')
    h_max = cv2.getTrackbarPos(f'C_{color}_H2', 'HSV Controls')
    s_min = cv2.getTrackbarPos(f'C_{color}_S1', 'HSV Controls')
    s_max = cv2.getTrackbarPos(f'C_{color}_S2', 'HSV Controls')
    v_min = cv2.getTrackbarPos(f'C_{color}_V1', 'HSV Controls')
    v_max = cv2.getTrackbarPos(f'C_{color}_V2', 'HSV Controls')
    
    return (np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max]))

def get_line_values():
    """获取直线检测参数"""
    canny1 = cv2.getTrackbarPos('Canny1', 'Line Controls')
    canny2 = cv2.getTrackbarPos('Canny2', 'Line Controls')
    hough_threshold = cv2.getTrackbarPos('HoughLines Threshold', 'Line Controls')
    
    return {
        'canny': {
            'threshold1': canny1,
            'threshold2': canny2
        },
        'hough_threshold': hough_threshold
    }

def get_lab_values():
    """获取LAB空间的阈值设置"""
    lab_params = {
        'gray': {
            'lower': np.array([
                cv2.getTrackbarPos('Gray_L_Min', 'LAB Controls'),
                cv2.getTrackbarPos('Gray_A_Min', 'LAB Controls'),
                cv2.getTrackbarPos('Gray_B_Min', 'LAB Controls')
            ]),
            'upper': np.array([
                cv2.getTrackbarPos('Gray_L_Max', 'LAB Controls'),
                cv2.getTrackbarPos('Gray_A_Max', 'LAB Controls'),
                cv2.getTrackbarPos('Gray_B_Max', 'LAB Controls')
            ])
        }
    }
    return lab_params

def get_ring_hsv_values(color):
    """获取圆环的HSV阈值"""
    h_min = cv2.getTrackbarPos(f'R_{color}_H1', 'Ring HSV Controls')
    h_max = cv2.getTrackbarPos(f'R_{color}_H2', 'Ring HSV Controls')
    s_min = cv2.getTrackbarPos(f'R_{color}_S1', 'Ring HSV Controls')
    s_max = cv2.getTrackbarPos(f'R_{color}_S2', 'Ring HSV Controls')
    v_min = cv2.getTrackbarPos(f'R_{color}_V1', 'Ring HSV Controls')
    v_max = cv2.getTrackbarPos(f'R_{color}_V2', 'Ring HSV Controls')
    
    return (np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max]))



def save_params():
    """保存HSV和LAB参数到JSON文件"""
    params = {
        'color': {
            'red': {'lower': [], 'upper': []},
            'green': {'lower': [], 'upper': []},
            'blue': {'lower': [], 'upper': []}
        },
        'line': {
            'canny': get_line_values()['canny'],
            'lab': get_lab_values()  # 只保存灰色LAB参数
        },
        'ring': {
            'red': {'lower': [], 'upper': []},
            'green': {'lower': [], 'upper': []},
            'blue': {'lower': [], 'upper': []}
        }
    }
    
    # 获取颜色HSV参数
    for color in ['red', 'green', 'blue']:
        lower, upper = get_hsv_values(color)
        params['color'][color]['lower'] = lower.tolist()
        params['color'][color]['upper'] = upper.tolist()
        
        lower, upper = get_ring_hsv_values(color)
        params['ring'][color]['lower'] = lower.tolist()
        params['ring'][color]['upper'] = upper.tolist()
    
    # 转换LAB参数中的ndarray为列表
    params['line']['lab']['gray']['lower'] = params['line']['lab']['gray']['lower'].tolist()
    params['line']['lab']['gray']['upper'] = params['line']['lab']['gray']['upper'].tolist()
    
    # 保存到JSON文件
    json_path = os.path.join(os.path.dirname(__file__), 'params.json')
    with open(json_path, 'w') as f:
        json.dump(params, f, indent=4)
    print("所有参数已保存")

def load_params():
    """从JSON文件加载所有参数"""
    json_path = os.path.join(os.path.dirname(__file__), 'params.json')
    if not os.path.exists(json_path):
        print("未找到参数文件，使用默认值")
        return
    
    with open(json_path, 'r') as f:
        params = json.load(f)
    
    # 设置HSV参数
    for color in ['red', 'green', 'blue']:
        # 设置物块颜色HSV参数
        lower = params['color'][color]['lower']
        upper = params['color'][color]['upper']
        cv2.setTrackbarPos(f'C_{color}_H1', 'HSV Controls', lower[0])
        cv2.setTrackbarPos(f'C_{color}_H2', 'HSV Controls', upper[0])
        cv2.setTrackbarPos(f'C_{color}_S1', 'HSV Controls', lower[1])
        cv2.setTrackbarPos(f'C_{color}_S2', 'HSV Controls', upper[1])
        cv2.setTrackbarPos(f'C_{color}_V1', 'HSV Controls', lower[2])
        cv2.setTrackbarPos(f'C_{color}_V2', 'HSV Controls', upper[2])
        
        # 设置圆环HSV参数
        lower = params['ring'][color]['lower']
        upper = params['ring'][color]['upper']
        cv2.setTrackbarPos(f'R_{color}_H1', 'Ring HSV Controls', lower[0])
        cv2.setTrackbarPos(f'R_{color}_H2', 'Ring HSV Controls', upper[0])
        cv2.setTrackbarPos(f'R_{color}_S1', 'Ring HSV Controls', lower[1])
        cv2.setTrackbarPos(f'R_{color}_S2', 'Ring HSV Controls', upper[1])
        cv2.setTrackbarPos(f'R_{color}_V1', 'Ring HSV Controls', lower[2])
        cv2.setTrackbarPos(f'R_{color}_V2', 'Ring HSV Controls', upper[2])
    
    # 设置Canny参数
    if 'canny' in params['line']:
        cv2.setTrackbarPos('Canny1', 'Line Controls', params['line']['canny']['threshold1'])
        cv2.setTrackbarPos('Canny2', 'Line Controls', params['line']['canny']['threshold2'])
    
    # 设置LAB参数
    if 'lab' in params['line']:
        lab = params['line']['lab']
        cv2.setTrackbarPos('Gray_L_Min', 'LAB Controls', lab['gray']['lower'][0])
        cv2.setTrackbarPos('Gray_L_Max', 'LAB Controls', lab['gray']['upper'][0])
        cv2.setTrackbarPos('Gray_A_Min', 'LAB Controls', lab['gray']['lower'][1])
        cv2.setTrackbarPos('Gray_A_Max', 'LAB Controls', lab['gray']['upper'][1])
        cv2.setTrackbarPos('Gray_B_Min', 'LAB Controls', lab['gray']['lower'][2])
        cv2.setTrackbarPos('Gray_B_Max', 'LAB Controls', lab['gray']['upper'][2])
    
    print("所有参数已加载")

def main():
    # 创建摄像头对象
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("摄像头打开失败")
        return
    
    # 设置摄像头分辨率为640x480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 创建HSV控制窗口和加载参数
    create_hsv_trackbars()
    load_params()  # 现在这一个函数就能加载所有参数
    
    # 创建预览窗口和遮罩窗口
    cv2.namedWindow('Preview', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Block Masks', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Ring Masks', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Line Mask', cv2.WINDOW_NORMAL)
    
    # 创建窗口最后更新时间字典
    window_last_update = {
        'Preview': time.time(),
        'Block Masks': time.time(),
        'Ring Masks': time.time(),
        'Line Mask': time.time()
    }
    
    while True:
        # 检查并关闭超时窗口
        current_time = time.time()
        for window_name, last_update in list(window_last_update.items()):
            if current_time - last_update > 3:  # 3秒超时
                cv2.destroyWindow(window_name)
                del window_last_update[window_name]
        
        ret, frame = cap.read()
        if not ret:
            break
            
        # 更新预览窗口时间
        cv2.imshow('Preview', frame)
        window_last_update['Preview'] = time.time()
        
        # 转换到HSV空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 处理物块颜色遮罩
        block_masks = []
        for color in ['red', 'green', 'blue']:
            lower, upper = get_hsv_values(color)
            mask = cv2.inRange(hsv, lower, upper)
            # 调整模糊核大小以适应更大的分辨率
            mask = cv2.GaussianBlur(mask, (15, 15), 2)
            mask = cv2.erode(mask, None, iterations=3)
            mask = cv2.dilate(mask, None, iterations=3)
            block_masks.append(mask)
        
        # 合并物块遮罩显示
        block_combined = np.vstack((block_masks[0], block_masks[1], block_masks[2]))
        cv2.imshow('Block Masks', block_combined)
        window_last_update['Block Masks'] = time.time()
        
        # 处理圆环遮罩
        ring_masks = []
        for color in ['red', 'green', 'blue']:
            lower, upper = get_ring_hsv_values(color)
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.erode(mask, None, iterations=0)
            mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.GaussianBlur(mask, (9, 9), 2)
            ring_masks.append(mask)
        
        # 合并圆环遮罩显示
        ring_combined = np.vstack((ring_masks[0], ring_masks[1], ring_masks[2]))
        cv2.imshow('Ring Masks', ring_combined)
        window_last_update['Ring Masks'] = time.time()
        
        # 处理直线遮罩
        line_params = get_line_values()
        
        # 转换到LAB空间和灰度图
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 获取当前LAB阈值
        lab_params = get_lab_values()
        
        # 创建LAB掩码
        lab_mask = cv2.inRange(lab, lab_params['gray']['lower'], lab_params['gray']['upper'])
        
        # 分别进行Canny边缘检测
        lab_edges = cv2.Canny(lab_mask, 
                         line_params['canny']['threshold1'], 
                         line_params['canny']['threshold2'])
        
        gray_edges = cv2.Canny(gray,
                          line_params['canny']['threshold1'],
                          line_params['canny']['threshold2'])
        
        # 分别进行霍夫直线检测
        lab_lines = cv2.HoughLines(lab_edges, 1, np.pi/180, line_params['hough_threshold'])
        gray_lines = cv2.HoughLines(gray_edges, 1, np.pi/180, line_params['hough_threshold'])
        
        # 创建可视化图像
        lab_vis = frame.copy()
        gray_vis = frame.copy()
        
        # 在两个图像上分别绘制检测到的线
        if lab_lines is not None:
            for rho, theta in lab_lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                
                cv2.line(lab_vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
                angle = np.arctan2(-(y2 - y1), x2 - x1) * 180.0 / np.pi
                cv2.putText(lab_vis, f"LAB Angle: {angle:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        if gray_lines is not None:
            for rho, theta in gray_lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                
                cv2.line(gray_vis, (x1, y1), (x2, y2), (255, 0, 0), 2)
                angle = np.arctan2(-(y2 - y1), x2 - x1) * 180.0 / np.pi
                cv2.putText(gray_vis, f"Gray Angle: {angle:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # 创建显示用的组合图像
        display_images = [
            cv2.cvtColor(lab_mask, cv2.COLOR_GRAY2BGR),    # LAB掩码
            cv2.cvtColor(gray_edges, cv2.COLOR_GRAY2BGR),  # 灰度边缘
            np.hstack((lab_vis, gray_vis))                 # 并排显示两种检测结果
        ]
        
        # 添加标签
        labels = ["LAB Mask", "Gray Edges", "Line Detection"]
        for img, label in zip(display_images[:2], labels[:2]):
            cv2.putText(img, label, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 垂直拼接显示
        line_combined = np.vstack([
            np.hstack((display_images[0], display_images[1])),
            display_images[2]
        ])
        cv2.imshow('Line Mask', line_combined)
        window_last_update['Line Mask'] = time.time()
        
        # 在预览图像上添加文本标签
        labels = ['Red', 'Green', 'Blue']
        for i, label in enumerate(labels):
            # 物块标签
            y_pos = 40 * (i + 1)  # 增加行间距
            cv2.putText(frame, f"Block {label}", (20, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)  # 增大字体
            # 圆环标签
            cv2.putText(frame, f"Ring {label}", (300, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        cv2.putText(frame, "Line", (580, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('Preview', frame)
        
        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):  # 's'键保存参数
            save_params()  # 现在这一个函数就能保存所有参数
        elif key == 27:  # ESC键退出
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
