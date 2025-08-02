import serial
def send_data_via_serial(str1, str2, str3):
    """
    这个函数用于通过串口发送两个字节和三个字符串数据。

    参数：
    byte1：第一个字节数据。
    byte2：第二个字节数据。
    str1：第一个字符串。
    str2：第二个字符串。
    str3：第三个字符串。
    """
    # 打开串口设备 /dev/ttyAMA0，设置波特率为 115200
    ser = serial.Serial('/dev/ttyAMA0', 115200)
    # 创建要发送的数据，首先是两个字节数据组成的 bytearray，然后分别将三个字符串编码后连接起来
    data_to_send = str1.encode() + str2.encode() + str3.encode()
    # 通过串口发送数据
    ser.write(data_to_send)
    # 关闭串口
    print(data_to_send)
send_data_via_serial('187','1','2')