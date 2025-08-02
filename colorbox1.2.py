import serial
import cv2
import numpy as np
import serial
import time
from pyzbar.pyzbar import decode
import json
import os

def safe_serial_init(port, baudrate, retries=3):
    for _ in range(retries):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            ser.dtr = 0  # 复位设备
            time.sleep(0.1)
            ser.dtr = 1
            time.sleep(0.5)  # 关键延时，等待硬件就绪
            ser.flushInput()
            return ser
        except serial.SerialException:
            time.sleep(1)
    raise Exception("Failed to initialize serial port")

# 添加圆环目标位置常量
RING_TARGET = {
    'x': 295,          # 目标X坐标
    'y': 265,          # 目标Y坐标
    'tolerance': 0    # 统一误差范围
}
# 添加时间控制变量
last_line_command_time = 0  # 上次发送命令的时间
LINE_COMMAND_INTERVAL = 0.8  # 命令发送间隔(秒)
last_ring_command_time = 0  # 上次发送圆环命令的时间
RING_COMMAND_INTERVAL = 0.5  # 圆环命令发送间隔(秒)# 添加目标距离常量

TARGET_ANGLE = 0  # 目标角度（度）
ANGLE_TOLERANCE = 0.1  # 角度允许的误差范围
TARGET_DISTANCE = 251  # 目标距离（像素）
DISTANCE_TOLERANCE = 2  # 距离允许的误差范围

# 添加窗口管理相关变量
active_windows = {}  # 存储窗口名称和最后活跃时间
WINDOW_TIMEOUT = 2.0  # 窗口超时时间（秒）

# 定义颜色范围常量
BOX_COLORS = {
    'red': (np.array([0, 142, 97]), np.array([13, 255, 255])),
    'green': (np.array([59, 77, 86]), np.array([99, 202, 187])),
    'blue': (np.array([94, 61, 86]), np.array([126, 185, 184])),
}
RING_COLORS = {
    'red': (np.array([0, 82, 75]), np.array([15, 215, 214])),   
    'green': (np.array([46, 69, 206]), np.array([105, 161, 255])), 
    'blue': (np.array([6, 81, 74]), np.array([43, 255, 255]))  
}
# 添加边线HSV范围常量（需要根据实际颜色调整）
LINE_HSV_RANGE = {
    'canny': {
        'threshold1': 73,
        'threshold2': 33
    }
}

# 添加边线方向常量
LINE_DIRECTIONS = {
    'A': {'angle': 0, 'name': 'FRONT'},     # 前边线
    'B': {'angle': 0, 'name': 'RIGHT'},    # 右边线
    'C': {'angle': 0, 'name': 'BACK'},    # 后边线
    'D': {'angle': 0, 'name': 'LEFT'}     # 左边线
}

# 修改圆环方向常量，移除单独的目标位置设置
RING_DIRECTIONS = {
    'E': {
        'name': 'FRONT',
        'move_x': {'pos': 'lm', 'neg': 'rm'},  # 左右移动
        'move_y': {'pos': 'ad', 'neg': 'rt'}   # 前后移动
    },
    'F': {
        'name': 'RIGHT',
        'move_x': {'pos': 'ad', 'neg': 'rt'},  # 前后移动
        'move_y': {'pos': 'rm', 'neg': 'lm'}   # 左右移动
    },
    'G': {
        'name': 'BACK',
        'move_x': {'pos': 'rm', 'neg': 'lm'},  # 左右移动
        'move_y': {'pos': 'rt', 'neg': 'ad'}   # 前后移动
    },
    'H': {
        'name': 'LEFT',
        'move_x': {'pos': 'rt', 'neg': 'ad'},  # 前后移动
        'move_y': {'pos': 'lm', 'neg': 'rm'}   # 左右移动
    }
}

# 添加颜色状态变量
color_old = None
qr_data_old = None  # 添加QR码状态变量

# 添加 ROI 相关常量
SCALE_FACTOR = 0.7
# 使用固定的ROI点位
ROI_DEFAULT_POINTS = {
    'top_left': (160, 240),
    'top_right': (480, 240),
    'bottom_right': (480, 480),
    'bottom_left': (160, 480)
}

# 添加ASCII命令常量
COMMANDS = {
    b'O': 'COLOR',    # ASCII 81 - 颜色识别
    b'Q': 'QR',       # ASCII 82 - QR码扫描
    b'S': 'STOP',     # ASCII 83 - 停止
    b'X': 'RESET',    # ASCII 88 - 重置
    b'A': 'FRONT',    # 前边巡线 (LAB方式)
    b'B': 'RIGHT',    # 右边巡线 (LAB方式)
    b'C': 'BACK',     # 后边巡线 (LAB方式)
    b'D': 'LEFT',     # 左边巡线 (LAB方式)
    b'a': 'FRONT_GRAY',    # 前边巡线 (灰度方式)
    b'b': 'RIGHT_GRAY',    # 右边巡线 (灰度方式)
    b'c': 'BACK_GRAY',     # 后边巡线 (灰度方式)
    b'd': 'LEFT_GRAY',     # 左边巡线 (灰度方式)
    b'E': 'FRONT_RING',    # 前边圆环校准
    b'F': 'RIGHT_RING',    # 右边圆环校准
    b'G': 'BACK_RING',     # 后边圆环校准
    b'H': 'LEFT_RING'      # 左边圆环校准
}

# 添加直线校准状态常量
LINE_CALIBRATION_STATES = {
    'DONE': 2  # 只保留完成状态
}
# 添加颜色预判断阈值常量
RING_DETECTION_THRESHOLD = 2000  # 颜色检测阈值
# 添加校准参数
current_calibration_state = LINE_CALIBRATION_STATES['DONE']
# 添加LAB参数全局变量
lab_params = None

# 添加PID控制器类
class PIDController:
    def __init__(self, kp, ki, kd, windup_guard=20.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup_guard = windup_guard
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return 0
        
        # 计算积分项
        self.integral += error * dt
        # 抗积分饱和
        if self.integral > self.windup_guard:
            self.integral = self.windup_guard
        elif self.integral < -self.windup_guard:
            self.integral = -self.windup_guard
        
        # 计算微分项
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        
        # 计算PID输出
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # 更新状态
        self.last_error = error
        self.last_time = current_time
        
        return output

# 初始化PID控制器（在全局变量区域）
pid_x = PIDController(kp=1.3, ki=0.1, kd=0.4)
pid_y = PIDController(kp=1.8, ki=0.1, kd=0.3)

def load_params():
    """从JSON文件加载所有参数，包括HSV和LAB参数"""
    global BOX_COLORS, LINE_HSV_RANGE, RING_COLORS, lab_params
    
    json_path = os.path.join(os.path.dirname(__file__), 'params.json')
    if not os.path.exists(json_path):
        print("未找到参数文件，使用默认参数")
        lab_params = {
            'gray': {
                'lower': np.array([100, 103, 103]),
                'upper': np.array([150, 127, 127])
            }
        }
        return lab_params
    
    try:
        with open(json_path, 'r') as f:
            params = json.load(f)
        
        # 更新颜色范围参数
        for color in ['red', 'green', 'blue']:
            lower = np.array(params['color'][color]['lower'])
            upper = np.array(params['color'][color]['upper'])
            BOX_COLORS[color] = (lower, upper)
        
        # 更新直线检测参数
        LINE_HSV_RANGE['canny'] = params['line']['canny']
        
        # 更新圆环检测参数
        for color in ['red', 'green', 'blue']:
            lower = np.array(params['ring'][color]['lower'])
            upper = np.array(params['ring'][color]['upper'])
            RING_COLORS[color] = (lower, upper)
        
        # 处理LAB参数并存储到全局变量
        lab_params = params['line']['lab']
        for color in lab_params:
            lab_params[color]['lower'] = np.array(lab_params[color]['lower'])
            lab_params[color]['upper'] = np.array(lab_params[color]['upper'])
        
        print("所有参数已加载")
        return lab_params
        
    except Exception as e:
        print(f"加载参数出错: {e}，使用默认参数")
        lab_params = {
            'gray': {
                'lower': np.array([100, 103, 103]),
                'upper': np.array([150, 127, 127])
            }
        }
        return lab_params

def map_control_to_pulses(angle):
    """将角度映射为步进电机脉冲数"""
    max_pulse = 3095

    pulse = int(abs(angle) * max_pulse / 90)  # 将角度按比例映射到脉冲数
    return min(max_pulse, max(0, pulse))

def map_distance_to_pulses(pixel_diff):
    """将像素差距映射为步进电机脉冲数基准：后退100个脉冲对应35像素的移动"""
    pulses = int(abs(pixel_diff) * 100 / 35)  # 将像素差距按比例映射到脉冲数
    # 限制脉冲数在50-500之间
    return min(500, max(5, pulses))

def format_pulse_str(pulse):
    """格式化脉冲数为5位数字字符串"""
    return f"{pulse:05d}"

def show_window(name, image):
    """显示窗口并更新时间戳"""
    cv2.imshow(name, image)
    active_windows[name] = time.time()

def check_and_close_windows():
    """检查并关闭超时的窗口"""
    current_time = time.time()
    closed_windows = []
    for window_name, last_active in active_windows.items():
        if current_time - last_active > WINDOW_TIMEOUT:
            cv2.destroyWindow(window_name)
            closed_windows.append(window_name)
    
    # 从活跃窗口列表中移除已关闭的窗口
    for window in closed_windows:
        active_windows.pop(window)

def process_qr_code(frame):
    global qr_data_old
    print("Processing QR Code...")
    # 进行QR码扫描
    decoded_objects = decode(frame)
    for obj in decoded_objects:
        qr_data = obj.data.decode('utf-8')
        if qr_data != qr_data_old:  # 只在QR码数据发生变化时发送
            print(f"QR Code detected: {qr_data}")
            qr_data_old = qr_data
            return qr_data
    return None

def process_box_color(frame):
    """处理物块颜色识别"""
    global color_old
    print("Processing box Color...")
    
    # 计算ROI点位坐标
    tl = (int(ROI_DEFAULT_POINTS['top_left'][0] * SCALE_FACTOR), 
          int(ROI_DEFAULT_POINTS['top_left'][1] * SCALE_FACTOR))
    tr = (int(ROI_DEFAULT_POINTS['top_right'][0] * SCALE_FACTOR), 
          int(ROI_DEFAULT_POINTS['top_right'][1] * SCALE_FACTOR))
    br = (int(ROI_DEFAULT_POINTS['bottom_right'][0] * SCALE_FACTOR), 
          int(ROI_DEFAULT_POINTS['bottom_right'][1] * SCALE_FACTOR))
    bl = (int(ROI_DEFAULT_POINTS['bottom_left'][0] * SCALE_FACTOR), 
          int(ROI_DEFAULT_POINTS['bottom_left'][1] * SCALE_FACTOR))
    
    # 在原始图像上绘制ROI边界和标记点
    disp_pts = np.array([ROI_DEFAULT_POINTS['top_left'],
                        ROI_DEFAULT_POINTS['top_right'],
                        ROI_DEFAULT_POINTS['bottom_right'],
                        ROI_DEFAULT_POINTS['bottom_left']], np.int32)
    disp_pts = disp_pts.reshape((-1,1,2))
    frame_with_roi = frame.copy()
    cv2.polylines(frame_with_roi, [disp_pts], True, (0,255,255), 2)  # 绘制黄色边界
    
    # 在ROI角点处添加红色圆点标记
    for pt in ROI_DEFAULT_POINTS.values():
        cv2.circle(frame_with_roi, pt, 3, (0,0,255), -1)
    
    # 显示带有ROI的原始图像
    show_window('Video', frame_with_roi)
    
    # 图像预处理：缩放
    processed = cv2.resize(frame, (0, 0), fx=SCALE_FACTOR, fy=SCALE_FACTOR)
    
    # 使用高斯滤波降噪
    processed = cv2.GaussianBlur(processed, (5, 5), 0)
    
    # 转换到HSV空间并自适应调整
    hsv = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)
    
    # ROI 掩码处理
    roi_mask = np.zeros(processed.shape[:2], dtype=np.uint8)
    pts = np.array([tl, tr, br, bl], np.int32)
    pts = pts.reshape((-1,1,2))
    cv2.fillPoly(roi_mask, [pts], 255)

    detected_color = None
    # 使用滑动条的值进行颜色检测
    for color in ['red', 'green', 'blue']:
        # 获取当前颜色的HSV值
        lower, upper = BOX_COLORS[color]
        
        # 创建颜色掩码并应用ROI
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.bitwise_and(mask, mask, mask=roi_mask)
        
        # 增强掩码处理
        mask = cv2.GaussianBlur(mask, (9, 9), 2)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # 显示每个颜色的mask
        show_window(f'{color} Mask', mask)
        
        # 使用自适应阈值检测颜色
        non_zero_count = cv2.countNonZero(mask)
        threshold = 15000
        
        if non_zero_count > threshold:
            if color != color_old:
                print(f"检测到{color}色，像素数量：{non_zero_count}")
                detected_color = color
                color_old = color
                time.sleep(0.5)  # 延时以避免重复发送相同颜色

    return detected_color

def process_ring(frame, direction=None):
    """处理圆环校准，必须提供direction参数"""
    global last_ring_command_time
    
    print(f"Processing Ring  - {RING_DIRECTIONS[direction]['name']}...")
    visual_frame = frame.copy()
    
    # 添加预判断逻辑
    # 在HSV转换前添加高斯模糊
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # 获取蓝色和红色的mask
    blue_lower, blue_upper = RING_COLORS['blue']
    red_lower, red_upper = RING_COLORS['red']
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    red_mask = cv2.inRange(hsv, red_lower, red_upper)
    # 显示预判断mask
    show_window('Blue Pre-detection', blue_mask)
    show_window('Red Pre-detection', red_mask)
    # 计算mask中非零像素的数量
    blue_count = cv2.countNonZero(blue_mask)
    red_count = cv2.countNonZero(red_mask)
    # 预判断阈值
    pre_threshold = RING_DETECTION_THRESHOLD
    current_time = time.time()
    # 获取当前方向的移动映射
    current_direction = RING_DIRECTIONS[direction]
    move_x = current_direction['move_x']  # 获取当前边的X方向移动映射
    # 如果检测到足够多的蓝色或红色像素，根据当前边的方向进行相应移动
    if (current_time - last_ring_command_time) >= RING_COMMAND_INTERVAL:
        if blue_count > pre_threshold:
            print(f"检测到大量蓝色 ({blue_count} 像素)，使用{move_x['neg']}命令移动")
            last_ring_command_time = current_time
            return f"Z{move_x['neg']}00600"  # 使用映射的负方向移动命令
        elif red_count > pre_threshold:
            print(f"检测到大量红色 ({red_count} 像素)，使用{move_x['pos']}命令移动")
            last_ring_command_time = current_time
            return f"Z{move_x['pos']}00600"  # 使用映射的正方向移动命令
    
    # 如果没有检测到足够的颜色或已经移动完成，执行原有的圆环校准逻辑
    current_direction = RING_DIRECTIONS[direction]
    # 使用统一的目标位置和容差
    target_x = RING_TARGET['x']
    target_y = RING_TARGET['y']
    tolerance = RING_TARGET['tolerance']
    # 绘制目标位置十字线
    cv2.line(visual_frame, (target_x-20, target_y), 
             (target_x+20, target_y), (255,255,0), 1)
    cv2.line(visual_frame, (target_x, target_y-20), 
             (target_x, target_y+20), (255,255,0), 1)
    
    # 使用圆环颜色范围进行检测
    for color_name in ['green']:
        lower, upper = RING_COLORS[color_name]
        mask = cv2.inRange(hsv, lower, upper)
        # 边缘检测
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, kernel, iterations=6)
        show_window(f'Edges_{color_name}', mask)
        # 查找轮廓
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            # 按面积筛选轮廓
            filtered_cnts = []
            for c in cnts:
                area = cv2.contourArea(c)
                if 5000 < area < 250000:
                    filtered_cnts.append(c)
            if filtered_cnts:
                # 找到最大轮廓
                c = max(filtered_cnts, key=cv2.contourArea)
                # 获取最小外接矩形
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int32(box)
                # 获取矩形的中心点、宽度、高度和角度
                center = (int(rect[0][0]), int(rect[0][1]))
                width = int(rect[1][0])
                height = int(rect[1][1])
                angle = rect[2]
                # 计算矩形的宽高比
                aspect_ratio = width / float(height) if height != 0 else 0
                # 判断是否为近似正方形（圆环的外接矩形应该接近正方形）
                if 0.8 <= aspect_ratio <= 1.2:
                    # 绘制检测结果
                    cv2.drawContours(visual_frame, [c], -1, (0,255,0), 2)  # 绘制轮廓
                    cv2.drawContours(visual_frame, [box], 0, (255,0,0), 2)  # 绘制最小外接矩形
                    cv2.circle(visual_frame, center, 5, (0,0,255), -1)  # 绘制中心点
                    # 显示信息
                    info_text = [
                        f"Color: {color_name}",
                        f"Center: ({center[0]}, {center[1]})",
                        f"Size: {width}x{height}",
                        f"Ratio: {aspect_ratio:.2f}"
                    ]
                    
                    # 直接使用中心点坐标进行偏差计算
                    x_diff = center[0] - target_x
                    y_diff = center[1] - target_y
                    
                    # 命令生成逻辑
                    command = None
                    current_time = time.time()
                    if (current_time - last_ring_command_time) >= RING_COMMAND_INTERVAL:
                        if abs(x_diff) > tolerance:
                            # X方向PID控制
                            pid_output = pid_x.compute(x_diff)
                            pulses = min(500, max(5, int(abs(pid_output))))
                            direction_cmd = current_direction['move_x']['neg'] if x_diff > 0 else current_direction['move_x']['pos']
                            command = f"{direction_cmd}{format_pulse_str(pulses)}0100050"
                        elif abs(y_diff) > tolerance:
                            # Y方向PID控制
                            pid_output = pid_y.compute(y_diff)
                            pulses = min(500, max(5, int(abs(pid_output))))
                            direction_cmd = current_direction['move_y']['neg'] if y_diff > 0 else current_direction['move_y']['pos']
                            command = f"{direction_cmd}{format_pulse_str(pulses)}0100050"
                        else:
                            command = "Done"
                            # 重置PID控制器
                            pid_x.integral = 0
                            pid_y.integral = 0
                        
                        if command:
                            last_ring_command_time = current_time
                            return f"Z{command}" if command != "Done" else command
    
    show_window('Ring Detection', visual_frame)
    return None

def process_line(frame, target_direction=None, use_gray=False):
    """处理直线检测
    target_direction: 目标边线方向（A/B/C/D）
    use_gray: 是否使用灰度图处理方式
    """
    global last_line_command_time, TARGET_ANGLE
    
    if target_direction and target_direction.upper() in LINE_DIRECTIONS:
        TARGET_ANGLE = LINE_DIRECTIONS[target_direction.upper()]['angle']
        print(f"当前巡线方向: {LINE_DIRECTIONS[target_direction.upper()]['name']}")
    
    vis_frame = frame.copy()
    
    if use_gray:
        # 使用灰度图和边缘检测方式
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,
                         LINE_HSV_RANGE['canny']['threshold1'], 
                         LINE_HSV_RANGE['canny']['threshold2'])
    else:
        # 使用LAB空间方式，直接使用全局lab_params
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        gray_mask = cv2.inRange(lab, lab_params['gray']['lower'], lab_params['gray']['upper'])
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        enhanced_mask = cv2.morphologyEx(gray_mask, cv2.MORPH_CLOSE, kernel)
        edges = cv2.Canny(enhanced_mask,50,150)

    # 使用霍夫直线检测
    lines = cv2.HoughLines(edges, 1, np.pi/180, 118)
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
            
            # 重新计算角度：使用atan2计算直线与水平方向的夹角
            # 注意：OpenCV的坐标系y轴向下，所以需要反转y的差值
            dy = -(y2 - y1)  # 反转y差值使其符合数学坐标系
            dx = x2 - x1
            angle = np.arctan2(dy, dx) * 180.0 / np.pi
            
            # 确保角度在-90到90度之间
            if angle > 90:
                angle = angle - 180
            elif angle < -90:
                angle = angle + 180
            
            cv2.putText(vis_frame, f"Angle: {angle:.1f}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 计算直线与图像边界的交点
            height, width = frame.shape[:2]
            # 直线方程: y = mx + c
            if x2 != x1:  # 避免除以零
                m = (y2 - y1) / (x2 - x1)
                c = y1 - m * x1
                
                # 找出与图像四边的交点
                intersections = []
                
                # 与左边界交点 (x=0)
                y_left = int(c)
                if 0 <= y_left <= height:
                    intersections.append((0, y_left))
                
                # 与右边界交点 (x=width)
                y_right = int(m * width + c)
                if 0 <= y_right <= height:
                    intersections.append((width, y_right))
                
                # 与上边界交点 (y=0)
                if m != 0:  # 避免除以零
                    x_top = int(-c / m)
                    if 0 <= x_top <= width:
                        intersections.append((x_top, 0))
                
                # 与下边界交点 (y=height)
                x_bottom = int((height - c) / m)
                if 0 <= x_bottom <= width:
                    intersections.append((x_bottom, height))
                
                # 选择最远的两个交点
                if len(intersections) >= 2:
                    # 计算这些点的平均值作为中点
                    mid_x = sum(x for x, _ in intersections) // len(intersections)
                    mid_y = sum(y for _, y in intersections) // len(intersections)
            else:
                # 处理垂直线的情况
                mid_x = x1
                mid_y = height // 2
            
            # 绘制中点（改为绿色）
            cv2.circle(vis_frame, (mid_x, mid_y), 5, (0, 255, 0), -1)

            # 计算并显示中点到底边的距离
            bottom_distance = height - mid_y
            
            # 绘制中点到底边的垂直线（使用相同的绿色）
            cv2.line(vis_frame, (mid_x, mid_y), (mid_x, height), (0, 255, 0), 1)
            
            cv2.putText(vis_frame, f"Distance: {bottom_distance}px", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 绘制检测到的线
            cv2.line(vis_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            print(f"Angle: {angle:.1f}")  # 打印时也保留一位小数
            
            # 命令生成逻辑
            command = None
            current_time = time.time()
            if (current_time - last_line_command_time) >= LINE_COMMAND_INTERVAL:
                # 优先处理角度调整
                if abs(angle - TARGET_ANGLE) > ANGLE_TOLERANCE:
                    pulses = map_control_to_pulses(abs(angle - TARGET_ANGLE))
                    direction = "aw" if angle > TARGET_ANGLE else "cw"
                    command = f"Z{direction}{format_pulse_str(pulses)}"
                # 角度在容差范围内，处理距离调整
                elif abs(bottom_distance - TARGET_DISTANCE) > DISTANCE_TOLERANCE:
                    pixel_diff = bottom_distance - TARGET_DISTANCE
                    pulses = map_distance_to_pulses(abs(pixel_diff))
                    
                    # 根据不同边线方向生成对应的运动指令
                    # 统一使用大写方向进行处理
                    target_direction = target_direction.upper()
                    if target_direction == 'A':  # 前边线
                        direction = "ad" if pixel_diff > 0 else "rt"
                    elif target_direction == 'B':  # 右边线
                        direction = "rm" if pixel_diff > 0 else "lm"
                    elif target_direction == 'C':  # 后边线
                        direction = "rt" if pixel_diff > 0 else "ad"
                    elif target_direction == 'D':  # 左边线
                        direction = "lm" if pixel_diff > 0 else "rm"
                    
                    command = f"Z{direction}{format_pulse_str(pulses)}"
                # 角度和距离都在容差范围内
                else:
                    command = "Done"
                
                if command:
                    print(f"发送命令: {command} (角度: {angle:.1f}°, 距离: {bottom_distance}px)")
                    last_line_command_time = current_time

            # 显示结果
            show_window('Line Detection', vis_frame)
            show_window('Edges', edges)
                        
            return command
    
    return None

def main():
    global color_old, qr_data_old, pid_x, pid_y, lab_params
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("摄像头初始化失败！")
        exit()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    try:
        ser = safe_serial_init('/dev/ttyAMA0', 9600)
    except Exception as e:
        print(f"串口初始化失败: {e}")
        exit()
    
    # 加载保存的参数（只加载一次）
    load_params()
    
    while True:
        # 读取摄像头图像
        ret, frame = cap.read()
        if not ret:
            break
        
        # 显示实时图像
        show_window('Video', frame)

        # 修改串口命令处理部分
        if ser.in_waiting > 0:
            data = ser.read(1)  # 读取一个字节的ASCII码
            if data in COMMANDS:
                if COMMANDS[data] == 'QR':
                    qr_result = process_qr_code(frame)
                    if qr_result:
                        ser.write(f"Q{qr_result}\r\n".encode())
                elif COMMANDS[data] == 'COLOR':
                    box_color_result = process_box_color(frame)
                    if box_color_result:
                        ser.write(f"C{box_color_result}\r\n".encode())
                elif data in [b'A', b'B', b'C', b'D']:  # 大写巡线命令，使用LAB
                    line_result = process_line(frame, data.decode(), use_gray=False)
                    if line_result:
                        ser.write(f"{line_result}\r\n".encode())
                elif data in [b'a', b'b', b'c', b'd']:  # 小写巡线命令，使用灰度图
                    line_result = process_line(frame, data.decode(), use_gray=True)
                    if line_result:
                        ser.write(f"{line_result}\r\n".encode())
                elif data in [b'E', b'F', b'G', b'H']:  # 圆环校准命令
                    ring_result = process_ring(frame, data.decode())
                    if ring_result:
                        print(f"发送圆环校准命令: {ring_result}")  # 添加此行打印命令
                        ser.write(f"{ring_result}\r\n".encode())
                elif COMMANDS[data] == 'RESET':
                    color_old = None
                    qr_data_old = None
                    print("Reset command received")
                elif COMMANDS[data] == 'STOP':
                    print("Stop command received")
            else:
                print(f"Unknown command: {data}")
                
        # 检查并关闭超时窗口
        check_and_close_windows()
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC键退出
            break
    
    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()

