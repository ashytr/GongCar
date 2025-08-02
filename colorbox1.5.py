import serial
import cv2
import numpy as np
import serial
import time
from pyzbar.pyzbar import decode
import json
import os

# 添加圆环目标位置常量# 目标X坐标# 目标Y坐标# 统一误差范围
RING_TARGET = {'x': 278,'y': 265,'tolerance': 1}
X_PID_PARA = {'kp': 1.44, 'ki': 0.1, 'kd': 0.4}  # PID参数
Y_PID_PARA = {'kp': 1.86, 'ki': 0.1, 'kd': 0.3}  # PID参数
# 添加时间控制变量
last_line_command_time = 0  # 上次发送命令的时间
LINE_COMMAND_INTERVAL = 0.5  # 命令发送间隔(秒)
last_ring_command_time = 0  # 上次发送圆环命令的时间
RING_COMMAND_INTERVAL = 0.5  # 圆环命令发送间隔(秒)
last_follow_command_time = 0  # 上次发送跟随命令的时间
FOLLOW_COMMAND_INTERVAL = 0.5  # 跟随命令发送间隔(秒)# 添加目标距离常量

# 在全局变量区域添加
last_target_x = None
last_movement_time = time.time()
STATIC_THRESHOLD = 5  # 物块位置变化阈值(像素)
STATIC_DURATION = 0.5  # 静止持续时间阈值(秒)
# 添加窗口管理相关变量
active_windows = {}  # 存储窗口名称和最后活跃时间
WINDOW_TIMEOUT = 2.0  # 窗口超时时间（秒）
# 添加 ROI 相关常量
SCALE_FACTOR = 0.7
# 使用固定的ROI点位
ROI_DEFAULT_POINTS = {
    'top_left': (160, 240),'top_right': (480, 240),
    'bottom_right': (480, 480),'bottom_left': (160, 480)
}
# 添加颜色预判断阈值常量
RING_DETECTION_THRESHOLD = 2000  # 颜色检测阈值
# 添加颜色状态变量
color_old = None
qr_data_old = None  # 添加QR码状态变量
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
    'canny': {'threshold1': 73,'threshold2': 33}
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

# 添加四边跟随方向常量
FOLLOW_DIRECTIONS = {
    'i': {
        'name': 'FRONT',
        'move_x': {'pos': 'lm', 'neg': 'rm'},  # 左右移动
        'move_y': {'pos': 'ad', 'neg': 'rt'}   # 前后移动
    },
    'j': {
        'name': 'RIGHT',
        'move_x': {'pos': 'ad', 'neg': 'rt'},  # 前后移动
        'move_y': {'pos': 'rm', 'neg': 'lm'}   # 左右移动
    },
    'k': {
        'name': 'BACK',
        'move_x': {'pos': 'rm', 'neg': 'lm'},  # 左右移动
        'move_y': {'pos': 'rt', 'neg': 'ad'}   # 前后移动
    },
    'l': {
        'name': 'LEFT',
        'move_x': {'pos': 'rt', 'neg': 'ad'},  # 前后移动
        'move_y': {'pos': 'lm', 'neg': 'rm'}   # 左右移动
    }
}

# 添加QR码跟随相关常量（添加在其他常量定义区域）
QR_TARGET = {
    'x': 280,          # 目标X坐标
    'tolerance': 10    # 允许的误差范围
}
QR_STABLE_THRESHOLD = 5  # 判定静止的位置误差范围(像素)
QR_STABLE_DURATION = 0.5  # 静止持续时间阈值(秒)

# 添加QR码位置追踪变量（添加在其他全局变量区域）
qr_last_position = None  # 上一次二维码位置
qr_last_movement_time = time.time()  # 上一次移动时间

# 添加ASCII命令常量
COMMANDS = {
    b'O': 'COLOR',    # ASCII 81 - 颜色识别
    b'Q': 'QR',       # ASCII 82 - QR码扫描
    b'S': 'STOP',     # ASCII 83 - 停止
    b'X': 'RESET',    # ASCII 88 - 重置
    b'i': 'FRONT_FOLLOW',   # 前边跟随
    b'j': 'RIGHT_FOLLOW',   # 右边跟随
    b'k': 'BACK_FOLLOW',    # 后边跟随
    b'l': 'LEFT_FOLLOW',    # 左边跟随
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
    b'H': 'LEFT_RING',      # 左边圆环校准
    b'e': 'BOX_BACK_RING'     # 后边物块圆环校准   
}

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

# 添加移动平均滤波器类
class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.x_values = []
        self.y_values = []
    
    def update(self, x, y):
        self.x_values.append(x)
        self.y_values.append(y)
        
        if len(self.x_values) > self.window_size:
            self.x_values.pop(0)
            self.y_values.pop(0)
        
        return self.get_average()
    
    def get_average(self):
        if not self.x_values or not self.y_values:
            return None, None
        return (sum(self.x_values) / len(self.x_values),
                sum(self.y_values) / len(self.y_values))
    
    def reset(self):
        self.x_values.clear()
        self.y_values.clear()

# 初始化PID控制器（在全局变量区域）
pid_x = PIDController(kp=X_PID_PARA['kp'], ki=X_PID_PARA['ki'], kd=X_PID_PARA['kd'])
pid_y = PIDController(kp=Y_PID_PARA['kp'], ki=Y_PID_PARA['ki'], kd=Y_PID_PARA['kd'])

# 初始化移动平均滤波器
ring_filter = MovingAverageFilter(window_size=5)

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

def init_serial_ports():
    """初始化两个串口"""
    try:
        # 初始化输入串口 - /dev/ttyACM0（用于接收命令）
        ser1 = safe_serial_init('/dev/ttyACM0', 9600)
        # 初始化输出串口 - /dev/ttyAMA0（用于发送QR码数据）
        ser2 = safe_serial_init('/dev/ttyAMA0', 9600)
        return ser1, ser2
    except Exception as e:
        print(f"串口初始化失败: {e}")
        return None, None

def load_params():
    """从JSON文件加载所有参数，包括HSV和LAB参数"""
    global BOX_COLORS, LINE_HSV_RANGE, RING_COLORS
    
    json_path = os.path.join(os.path.dirname(__file__), 'params.json')
    if not os.path.exists(json_path):
        print("未找到参数文件，使用默认参数")
        return {
            'lab': {
                'gray': {
                    'lower': np.array([100, 103, 103]),
                    'upper': np.array([150, 127, 127])
                }
            }
        }
    
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
        
        # 处理LAB参数
        lab_params = params['line']['lab']
        for color in lab_params:
            lab_params[color]['lower'] = np.array(lab_params[color]['lower'])
            lab_params[color]['upper'] = np.array(lab_params[color]['upper'])
        
        print("所有参数已加载")
        return lab_params
        
    except Exception as e:
        print(f"加载参数出错: {e}，使用默认参数")
        return {
            'lab': {
                'gray': {
                    'lower': np.array([100, 103, 103]),
                    'upper': np.array([150, 127, 127])
                }
            }
        }

def map_control_to_pulses(angle):
    """将角度差距按比例映射到脉冲数基准：90度对应3120个脉冲"""
    pulse = int(abs(angle) * 3120 / 90)  # 将角度差距按比例映射到脉冲数
    return min(300, max(0, pulse))
def map_distance_to_pulses(pixel_diff):
    """将像素差距映射为步进电机脉冲数基准：后退100个脉冲对应40像素的移动"""
    pulses = int(abs(pixel_diff) * 100 / 40)  # 将像素差距按比例映射到脉冲数
    # 限制脉冲数在50-500之间
    return min(300, max(5, pulses))
def map_follow_pulses(pixel_diff):
    """将像素差距映射为步进电机脉冲数基准：100像素对应100个脉冲"""
    pulses = int(abs(pixel_diff) * 100 / 70)
    return min(300, max(5, pulses))
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
def process_qr_code(ser1,ser2):
    """处理QR码并通过指定串口发送结果"""
    global qr_data_old
    # 进行QR码扫描
    # 从串口1读取数据
    if ser1.in_waiting:
        data = ser1.readline()
        # 检查数据是否已经以'Q'开头
        decoded_data = data.decode().strip()
        if decoded_data != qr_data_old:  # 只在QR码数据发生变化时发送
            print("Processing QR Code...")
            if not decoded_data.startswith('Q'):
                modified_data = f'Q{decoded_data}'.encode()
            else:
                modified_data = data
            ser2.write(modified_data)
            print(f"转发数据: {modified_data.decode().strip()}")
        
        time.sleep(0.01)  # 短暂延时，避免CPU占用过高
        qr_data_old = decoded_data  # 更新QR码数据状态
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
    # 创建一个列表存储所有mask用于后续垂直组合
    all_masks = []
    all_mask_names = []
    
    # 进行颜色检测
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
        
        # 将mask转换为三通道以便添加颜色标签
        colored_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # 添加颜色标签
        cv2.putText(colored_mask, color, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 存储处理后的mask
        all_masks.append(colored_mask)
        all_mask_names.append(color)
        
        # 使用自适应阈值检测颜色
        non_zero_count = cv2.countNonZero(mask)
        threshold = 15000
        
        if non_zero_count > threshold:
            if color != color_old:
                print(f"检测到{color}色，像素数量：{non_zero_count}")
                detected_color = color
                color_old = color
                time.sleep(0.5)  # 延时以避免重复发送相同颜色
    
    # 垂直组合所有mask
    if all_masks:
        combined_masks = np.vstack(all_masks)
        # 显示组合后的masks
        show_window('Combined Color Masks', combined_masks)

    return detected_color

def process_qr_follow(frame, edge='i'):
    """处理二维码跟随功能，类似物块跟随的实现"""
    global qr_last_position, qr_last_movement_time, last_follow_command_time
    
    if edge not in FOLLOW_DIRECTIONS:
        print(f"Invalid follow direction: {edge}")
        return None
        
    direction = FOLLOW_DIRECTIONS[edge]
    print(f"Processing QR {direction['name']} Follow...")
    
    decoded_objects = decode(frame)
    for obj in decoded_objects:
        points = obj.polygon
        if len(points) > 0:
            # 计算二维码边界
            x_coords = [p.x for p in points]
            left_x = min(x_coords)
            right_x = max(x_coords)
            center_x = (left_x + right_x) // 2
            
            # 绘制二维码边框和标记点
            pts = np.array(points, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.circle(frame, (left_x, points[0].y), 5, (255, 0, 0), -1)  # 左边界点
            cv2.circle(frame, (right_x, points[0].y), 5, (255, 0, 0), -1)  # 右边界点
            cv2.circle(frame, (center_x, points[0].y), 5, (0, 0, 255), -1)  # 中心点
            
            # 绘制目标位置线
            cv2.line(frame, (QR_TARGET['x'], 0), (QR_TARGET['x'], frame.shape[0]), (0, 255, 255), 1)
            
            # 计算与目标位置的偏差
            diff_to_target = center_x - QR_TARGET['x']
            
            # 显示移动信息
            cv2.putText(frame, f"Target: {QR_TARGET['x']}, Current: {center_x}, Diff: {diff_to_target}px",
                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # 判断二维码是否静止
            current_time = time.time()
            is_static = False
            
            if qr_last_position is not None:
                position_diff = abs(center_x - qr_last_position)
                time_diff = current_time - qr_last_movement_time
                
                # 显示静止状态信息
                cv2.putText(frame, f"Position Change: {position_diff}px, Static Time: {time_diff:.1f}s",
                          (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                if position_diff < QR_STABLE_THRESHOLD:
                    if time_diff > QR_STABLE_DURATION:
                        is_static = True
                        cv2.putText(frame, "Static", (10, 90),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                else:
                    qr_last_movement_time = current_time
            
            qr_last_position = center_x
            
            # 只有在二维码静止时才生成移动命令
            if is_static:
                if (current_time - last_follow_command_time) >= FOLLOW_COMMAND_INTERVAL:
                    if abs(diff_to_target) > QR_TARGET['tolerance']:
                        pulses = map_follow_pulses(abs(diff_to_target))
                        # 根据边缘方向选择移动命令
                        if diff_to_target < 0:
                            move_cmd = direction['move_x']['pos']
                        else:
                            move_cmd = direction['move_x']['neg']
                        print(f"Send command: {move_cmd} {pulses} pulses, Adjust distance {abs(diff_to_target)} pixels")
                        last_follow_command_time = current_time
                        return f"Z{move_cmd}{format_pulse_str(pulses)}"
                    else:
                        print(f"QR {direction['name']} follow completed")
                        last_follow_command_time = current_time
                        return "Done"
            else:
                cv2.putText(frame, "Waiting for stable...", (10, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    
    # 如果没有检测到二维码，重置状态
    qr_last_position = None
    qr_last_movement_time = current_time
    cv2.putText(frame, "No QR code detected", (10, 120),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    show_window('QR Follow', frame)
    return None

def process_follow_block(frame, edge='i'):
    """处理物块跟随功能，根据物块位置控制车身左右移动"""
    global color_old, last_target_x, last_movement_time, last_follow_command_time
    
    if edge not in FOLLOW_DIRECTIONS:
        print(f"Invalid follow direction: {edge}")
        return None
        
    direction = FOLLOW_DIRECTIONS[edge]
    print(f"Processing {direction['name']} Follow...")
    
    # 计算下三分之一区域
    height, width = frame.shape[:2]
    roi_start = height * 2 // 3
    
    # 创建ROI区域
    roi = frame[roi_start:height, 0:width]
    cv2.line(frame, (0, roi_start), (width, roi_start), (0, 255, 255), 2)
    
    # 转换ROI到HSV空间
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    target_x = 280  # 设置目标位置为280
    tolerance = 10  # 允许的误差范围

    # 检测所有可能的颜色
    max_area = 0
    _detected_x = None
    detected_color = None
    
    for color in BOX_COLORS:
        lower, upper = BOX_COLORS[color]
        mask = cv2.inRange(hsv, lower, upper)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area and area > 1000:
                max_area = area
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    _detected_x = int(M["m10"] / M["m00"])
                    detected_color = color

    if _detected_x is not None:
        current_time = time.time()
        is_static = False
        
        # 显示中心线和目标线
        cv2.line(frame, (target_x, roi_start), (target_x, height), (0, 0, 255), 2)  # 目标线为红色
        cv2.line(frame, (_detected_x, roi_start), (_detected_x, height), (0, 255, 0), 2)  # 检测到的位置为绿色
        
        # 计算移动信息
        diff_to_target = _detected_x - target_x  # 修改为相对目标位置的差距
        movement_info = f"Target: {target_x}, Current: {_detected_x}, Diff: {diff_to_target}px"
        cv2.putText(frame, movement_info, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # 判断物块是否静止
        if last_target_x is not None:
            position_diff = abs(_detected_x - last_target_x)
            time_diff = current_time - last_movement_time
            
            # 显示静止状态信息
            status_info = f"Position Change: {position_diff}px, Static Time: {time_diff:.1f}s"
            cv2.putText(frame, status_info, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            if position_diff < STATIC_THRESHOLD:
                if time_diff > STATIC_DURATION:
                    is_static = True
            else:
                last_movement_time = current_time
        
        last_target_x = _detected_x
        
        if is_static:
            # 添加时间间隔控制
            if (current_time - last_follow_command_time) >= FOLLOW_COMMAND_INTERVAL:
                if abs(diff_to_target) > tolerance:
                    pulses = map_follow_pulses(abs(diff_to_target))
                    # 根据边缘方向选择移动命令
                    if diff_to_target < 0:
                        move_cmd = direction['move_x']['pos']
                    else:
                        move_cmd = direction['move_x']['neg']
                    print(f"Send command: {move_cmd} {pulses} pulses, Adjust distance {abs(diff_to_target)} pixels")
                    last_follow_command_time = current_time
                    return f"Z{move_cmd}{format_pulse_str(pulses)}"
                else:
                    print(f"{direction['name']} follow completed")
                    last_follow_command_time = current_time
                    return "Done"

        else:
            cv2.putText(frame, "Waiting for stable...", (10, roi_start-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    else:
        cv2.putText(frame, "No block detected", (10, roi_start-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    show_window('Follow Block', frame)
    return None


def process_ring(frame, direction=None):
    """处理圆环校准，必须提供direction参数"""
    global last_ring_command_time, ring_filter
    
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
        #show_window(f'Edges_{color_name}', mask)
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
                    
                    # 对中心点坐标进行移动平均滤波
                    filtered_x, filtered_y = ring_filter.update(center[0], center[1])
                    if filtered_x is not None and filtered_y is not None:
                        # 使用滤波后的坐标进行偏差计算
                        x_diff = filtered_x - target_x
                        y_diff = filtered_y - target_y
                        
                        # 在原图上显示原始中心点和滤波后的中心点
                        cv2.circle(visual_frame, center, 5, (0,0,255), -1)  # 原始中心点(红色)
                        cv2.circle(visual_frame, (int(filtered_x), int(filtered_y)), 5, (0,255,0), -1)  # 滤波后的中心点(绿色)
                        
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
                                # 重置PID控制器和滤波器
                                pid_x.integral = 0
                                pid_y.integral = 0
                                ring_filter.reset()
                            
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
        # 使用LAB空间方式
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        lab_params = load_params()
        gray_mask = cv2.inRange(lab, lab_params['gray']['lower'], lab_params['gray']['upper'])
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        enhanced_mask = cv2.morphologyEx(gray_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        edges = cv2.Canny(enhanced_mask,
                         LINE_HSV_RANGE['canny']['threshold1'],
                         LINE_HSV_RANGE['canny']['threshold2'])

    # 使用霍夫直线检测
    lines = cv2.HoughLines(edges, 1, np.pi/180, 120)
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
                if abs(angle - TARGET_ANGLE) > ANGLE_TOLERANCE1:
                    pulses = map_control_to_pulses(abs(angle - TARGET_ANGLE))
                    direction = "aw" if angle > TARGET_ANGLE else "cw"
                    command = f"Z{direction}{format_pulse_str(pulses)}"
                # 角度在容差范围内，处理距离调整
                elif abs(bottom_distance - TARGET_DISTANCE1) > DISTANCE_TOLERANCE1:
                    pixel_diff = bottom_distance - TARGET_DISTANCE1
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
    global color_old, qr_data_old, pid_x, pid_y
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("摄像头初始化失败！")
        exit()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 初始化两个串口
    ser1, ser2 = init_serial_ports()
    if ser1 is None or ser2 is None:
        print("串口初始化失败")
        exit()
    
    # 加载参数
    load_params()
    
    while True:
        # 读取摄像头图像
        ret, frame = cap.read()
        if not ret:
            break
        
        # 显示实时图像
        show_window('Video', frame)
        # 持续处理QR码
        process_qr_code(ser1, ser2)
        
        # 处理串口命令
        if ser2.in_waiting > 0:
            data = ser2.read(1)  # 读取一个字节的ASCII码
            if data in COMMANDS:
                if data == b'O':
                    box_color_result = process_box_color(frame)
                    if box_color_result:
                        ser2.write(f"C{box_color_result}\r\n".encode())
                elif data == b'M':  # 大写taioxing命令
                    qr_follow_result = process_qr_follow(frame,'j')
                    if qr_follow_result:
                        ser2.write(f"{qr_follow_result}\r\n".encode())
                elif data in [b'A', b'B', b'C', b'D']:  # 大写巡线命令，使用LAB
                    line_result = process_line(frame, data.decode(), use_gray=False)
                    if line_result:
                        ser2.write(f"{line_result}\r\n".encode())
                elif data in [b'a', b'b', b'c', b'd']:  # 小写巡线命令，使用灰度图
                    line_result = process_line(frame, data.decode(), use_gray=True)
                    if line_result:
                        ser2.write(f"{line_result}\r\n".encode())
                elif data in [b'i', b'j', b'k', b'l']:  # 四边跟随命令
                    follow_result = process_follow_block(frame, data.decode())
                    if follow_result:
                        ser2.write(f"{follow_result}\r\n".encode())
                elif data in [b'E', b'F', b'G', b'H']:  # 圆环校准命令
                    ring_result = process_ring(frame, data.decode())
                    if ring_result:
                        print(f"发送圆环校准命令: {ring_result}")  # 添加此行打印命令
                        ser2.write(f"{ring_result}\r\n".encode())
                elif data == b'X':
                    color_old = None
                    qr_data_old = None
                    ring_filter.reset()  # 重置滤波器
                    print("Reset command received")
                elif data == b'S':
                    print("Stop command received")
                else:
                    print(f"Unknown COMMANDS: {data}")
            else:
                print(f"Unknown command: {data}")
        
        # 检查并关闭超时窗口
        check_and_close_windows()
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC键退出
            break
    
    cap.release()
    cv2.destroyAllWindows()
    ser1.close()
    ser2.close()

if __name__ == "__main__":
    main()

