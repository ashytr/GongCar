import serial
import cv2
import numpy as np
import serial
import time
from pyzbar.pyzbar import decode
import json
import os

def process_qr_code(frame):
    global qr_data_old
    print("Processing QR Code...")
    
    obj = decode(frame)
    qr_data = obj.data.decode('utf-8')
    # 在二维码上方显示解码内容
    cv2.putText(frame, str(obj.data.decode('utf-8')), 
                (points[0].x, points[0].y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    if qr_data != qr_data_old:
        print(f"QR Code detected: {qr_data}")
        qr_data_old = qr_data
        return qr_data
    return None
    
    cv2.imshow('QR Code Detection', frame)
    return None, None

def process_qr_follow(frame, edge='i'):
    """处理二维码跟随功能，类似物块跟随的实现"""
    global qr_last_position, qr_last_movement_time, last_follow_command_time
    
    if edge not in FOLLOW_DIRECTIONS:
        print(f"Invalid follow direction: {edge}")
        return None
        
    direction = FOLLOW_DIRECTIONS[edge]
    print(f"Processing QR {direction['name']} Follow...")
    
    # 将current_time的定义移到函数开始处
    current_time = time.time()
    
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
    

    
    show_window('QR Follow', frame)
    return None
    
