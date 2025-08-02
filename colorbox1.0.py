import serial
import cv2
import numpy as np
import serial
import time
from pyzbar.pyzbar import decode
import json
import os

# 添加圆环统一目标位置常量
RING_TARGET = {
    'x': 286,          # 目标X坐标
    'y': 254,          # 目标Y坐标
    'tolerance': 2    # 统一误差范围
}
# 添加目标距离常量
TARGET_ANGLE = 0  # 目标角度（度）
ANGLE_TOLERANCE = 0.5  # 角度允许的误差范围
TARGET_DISTANCE = 251  # 目标距离（像素）
DISTANCE_TOLERANCE = 3  # 距离允许的误差范围

# 添加时间控制变量
last_command_time = 0  # 上次发送命令的时间
COMMAND_INTERVAL = 0.8  # 命令发送间隔(秒)
# 添加圆环命令时间控制变量
last_ring_command_time = 0  # 上次发送圆环命令的时间
RING_COMMAND_INTERVAL = 0.8  # 圆环命令发送间隔(秒)

# 修改QR码目标位置常量
QR_TARGET = {
    'x': 280,          # 目标X坐标（图像中心）
    'x_tolerance': 10,  # X方向允许的误差范围
}

# 添加二维码静止检测常量
QR_STABLE_TIME = 1.0  # 二维码需要保持静止的时间(秒)
QR_POSITION_TOLERANCE = 5  # 判定静止的位置误差范围(像素)

# 添加颜色状态变量
color_old = None
qr_data_old = None  # 添加QR码状态变量

# 添加二维码静止检测变量
qr_last_position = None  # 上一次二维码位置
qr_stable_start_time = 0  # 二维码开始静止的时间
qr_is_stable = False  # 二维码是否已经稳定

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
    b'C': 'COLOR',    # ASCII 81 - 颜色识别
    b'Q': 'QR',       # ASCII 82 - QR码扫描
    b'S': 'STOP',     # ASCII 83 - 停止
    b'X': 'RESET',    # ASCII 88 - 重置
    b'A': 'FRONT',    # 前边巡线
    b'B': 'RIGHT',    # 右边巡线
    b'C': 'BACK',     # 后边巡线
    b'D': 'LEFT',     # 左边巡线
    b'E': 'FRONT_RING',    # 前边圆环校准
    b'F': 'RIGHT_RING',    # 右边圆环校准
    b'G': 'BACK_RING',     # 后边圆环校准
    b'H': 'LEFT_RING',     # 左边圆环校准
    b'i': 'FRONT_DETECT',    # 前边检测
    b'k': 'BACK_DETECT',     # 后边检测
    b'j': 'LEFT_DETECT',     # 左边检测
    b'l': 'RIGHT_DETECT'     # 右边检测
}

# 添加直线校准状态常量
LINE_CALIBRATION_STATES = {
    'DONE': 2  # 只保留完成状态
}

# 添加校准参数
current_calibration_state = LINE_CALIBRATION_STATES['DONE']
# 添加圆环中心点缓冲区常量和变量
RING_CENTER_BUFFER_SIZE = 5  # 保存最近5个中心点
ring_center_buffer = {'x': [], 'y': []}  # 用于存储历史中心点

# 添加目标距离常量
TARGET_ANGLE = 0  # 目标角度（度）
ANGLE_TOLERANCE = 0.5  # 角度允许的误差范围
TARGET_DISTANCE = 251  # 目标距离（像素）
DISTANCE_TOLERANCE = 3  # 距离允许的误差范围

def map_follow_pulses(pixel_diff):
    """将像素差距映射为步进电机脉冲数基准：100像素对应100个脉冲"""
    pulses = int(abs(pixel_diff) * 100 / 70)
    return min(300, max(5, pulses))

def enhance_brightness(image, alpha=1.5, beta=10):
    """增强图像亮度
    alpha: 对比度增强因子(1.0-3.0)
    beta: 亮度增强值(0-100)
    """
    enhanced = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    return enhanced

def map_control_to_pulses(angle):
    """将角度映射为步进电机脉冲数"""
    max_pulse = 3095

    pulse = int(abs(angle) * max_pulse / 90)  # 将角度按比例映射到脉冲数
    return min(max_pulse, max(0, pulse))

def map_distance_to_pulses(pixel_diff):
    """将像素差距映射为步进电机脉冲数
    基准：后退100个脉冲对应35像素的移动
    """
    pulses = int(abs(pixel_diff) * 100 / 35)  # 将像素差距按比例映射到脉冲数
    # 限制脉冲数在50-500之间
    return min(500, max(5, pulses))

def format_pulse_str(pulse):
    """格式化脉冲数为5位数字字符串"""
    return f"{pulse:05d}"

def process_qr_code(frame):
    global qr_data_old, last_command_time, qr_last_position, qr_stable_start_time, qr_is_stable
    print("Processing QR Code...")
    
    decoded_objects = decode(frame)
    for obj in decoded_objects:
        points = obj.polygon
        if len(points) > 0:
            x_coords = [p.x for p in points]
            left_x = min(x_coords)
            right_x = max(x_coords)
            center_x = (left_x + right_x) // 2
            
            # 检查二维码是否静止
            current_time = time.time()
            if qr_last_position is None:
                qr_last_position = center_x
                qr_stable_start_time = current_time
            elif abs(center_x - qr_last_position) <= QR_POSITION_TOLERANCE:
                if not qr_is_stable and (current_time - qr_stable_start_time) >= QR_STABLE_TIME:
                    qr_is_stable = True
                    print("QR code is now stable")
            else:
                # 位置发生较大变化，重置静止检测
                qr_last_position = center_x
                qr_stable_start_time = current_time
                qr_is_stable = False
            
            # 计算与目标位置的偏差
            x_diff = center_x - QR_TARGET['x']
            
            # 绘制二维码边框和标记点（保持原有的可视化代码）
            pts = np.array(points, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.circle(frame, (left_x, points[0].y), 5, (255, 0, 0), -1)
            cv2.circle(frame, (right_x, points[0].y), 5, (255, 0, 0), -1)
            cv2.circle(frame, (center_x, points[0].y), 5, (0, 0, 255), -1)
            cv2.line(frame, (QR_TARGET['x'], 0), (QR_TARGET['x'], frame.shape[0]), (0,255,255), 1)
            
            # 显示状态信息
            status = "Stable" if qr_is_stable else f"Stabilizing: {current_time - qr_stable_start_time:.1f}s"
            cv2.putText(frame, f"Status: {status}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(frame, f"X-diff: {x_diff}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            
            # 只有在二维码稳定后才生成控制命令
            command = None
            if qr_is_stable:
                current_time = time.time()
                if (current_time - last_command_time) >= COMMAND_INTERVAL:
                    if abs(x_diff) > QR_TARGET['x_tolerance']:
                        direction = "ad" if x_diff > 0 else "rt"
                        pulses = map_follow_pulses(abs(x_diff))
                        command = f"Z{direction}{format_pulse_str(pulses)}"
                        print(f"Sending command: {command}")
                        last_command_time = current_time
            
            # 显示二维码内容（保持原有代码）
            qr_data = obj.data.decode('utf-8')
            cv2.putText(frame, qr_data, 
                      (left_x, points[0].y - 10),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow('QR Code Detection', frame)
            
            if qr_data != qr_data_old:
                print(f"QR Code detected: {qr_data}")
                qr_data_old = qr_data
                return qr_data, command
            return None, command
    
    # 如果没有检测到二维码，重置状态
    qr_last_position = None
    qr_stable_start_time = 0
    qr_is_stable = False
    return None, None

def load_hsv_params():
    """从JSON文件加载HSV参数"""
    global BOX_COLORS, LINE_HSV_RANGE, RING_COLORS
    
    json_path = os.path.join(os.path.dirname(__file__), 'hsv_params.json')
    if not os.path.exists(json_path):
        print("未找到HSV参数文件，使用默认值")
        return
    
    with open(json_path, 'r') as f:
        params = json.load(f)
    
    # 更新颜色范围参数
    for color in ['red', 'green', 'blue']:
        lower = np.array(params['color'][color]['lower'])
        upper = np.array(params['color'][color]['upper'])
        BOX_COLORS[color] = (lower, upper)
    
    # 更新直线检测参数
    LINE_HSV_RANGE['lower'] = np.array(params['line']['lower'])
    LINE_HSV_RANGE['upper'] = np.array(params['line']['upper'])
    
    # 更新圆环检测参数
    for color in ['red', 'green', 'blue']:
        lower = np.array(params['ring'][color]['lower'])
        upper = np.array(params['ring'][color]['upper'])
        RING_COLORS[color] = (lower, upper)
    
    print("HSV参数已加载")

def main():
    global color_old, qr_data_old, ring_center_buffer
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("摄像头初始化失败！")
        exit()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)  # 根据实际情况修改串口号/dev/ttyAMA0和波特率
    
    # 加载保存的HSV参数
    load_hsv_params()
    
    while True:
        # 读取摄像头图像
        ret, frame = cap.read()
        if not ret:
            break
        
        # 显示实时图像
        cv2.imshow('Video', frame)

        process_qr_code(frame)
        # 修改串口命令处理部分
        # if ser.in_waiting > 0:
        #     data = ser.read(1)  # 读取一个字节的ASCII码
        #     if data in COMMANDS:
        #         if COMMANDS[data] == 'QR':
        #             qr_result = process_qr_code(frame)
        #             if qr_result:
        #                 ser.write(f"Q{qr_result}\r\n".encode())
        #         elif data in [b'i', b'k', b'j', b'l']:  # 四边检测命令
        #             result = process_box_color(frame, data.decode())
        #             if result:
        #                 color, command = result
        #                 if command:
        #                     ser.write(f"{command}\r\n".encode())
        #                 else:
        #                     ser.write(f"C{color}\r\n".encode())
        #         elif COMMANDS[data] == 'COLOR':
        #             result = process_box_color(frame)
        #             if result:
        #                 color, command = result
        #                 if command:
        #                     ser.write(f"{command}\r\n".encode())
        #                 else:
        #                     ser.write(f"C{color}\r\n".encode())
        #         elif data in [b'A', b'B', b'C', b'D']:  # 巡线命令
        #             line_result = process_line(frame, data.decode())
        #             if line_result:
        #                 ser.write(f"{line_result}\r\n".encode())
        #         elif data in [b'E', b'F', b'G', b'H']:  # 圆环校准命令
        #             ring_result = process_ring(frame, data.decode())
        #             if ring_result:
        #                 print(f"发送圆环校准命令: {ring_result}")  # 添加此行打印命令
        #                 ser.write(f"{ring_result}\r\n".encode())
        #         elif COMMANDS[data] == 'RESET':
        #             color_old = None
        #             qr_data_old = None  # 重置QR码状态
        #             ring_center_buffer = {'x': [], 'y': []}  # 重置圆环中心点缓冲区
        #             print("Reset command received")
        #         elif COMMANDS[data] == 'STOP':
        #             print("Stop command received")
        #     else:
        #         print(f"Unknown command: {data}")
                
        # 按's'键保存参数
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC键退出
            break
    
    cap.release()
    cv2.destroyAllWindows()
    # ser.close()

if __name__ == "__main__":
    main()

