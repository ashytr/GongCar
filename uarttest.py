import serial
import time

try:
    # 配置串口1 - /dev/ttyACM0
    ser1 = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )

    # 配置串口2 - /dev/ttyAMA0
    ser2 = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )

    while True:
        # 从串口1读取数据
        if ser1.in_waiting:
            data = ser1.readline()
            # 检查数据是否已经以'Q'开头
            decoded_data = data.decode().strip()
            if not decoded_data.startswith('Q'):
                modified_data = f'Q{decoded_data}'.encode()
            else:
                modified_data = data
            ser2.write(modified_data)
            print(f"转发数据: {modified_data.decode().strip()}")
        
        time.sleep(0.01)  # 短暂延时，避免CPU占用过高

except serial.SerialException as e:
    print(f"串口错误: {e}")
except Exception as e:
    print(f"其他错误: {e}")
finally:
    # 关闭串口
    if 'ser1' in locals():
        ser1.close()
    if 'ser2' in locals():
        ser2.close()

