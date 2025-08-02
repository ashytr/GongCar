import cv2
import numpy as np
# 打开摄像头
cap = cv2.VideoCapture(1)

while True:
    # 读取一帧图像
    ret, frame = cap.read()
    
    # 绘制红色矩形框
    cv2.rectangle(frame, (175, 238), (399, 461), (0, 0, 255), 2)
    
    # 显示图像
    cv2.imshow('Camera', frame)

    # 按下空格键拍照并分析绿色的 HSV 值
    if cv2.waitKey(1) == ord(' '):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 在图像中选择一个绿色区域（可以手动选择一个区域）
        roi = hsv[238:461, 175:399]

        # 计算所选区域的 HSV 均值
        h_mean = np.mean(roi[:, :, 0])
        s_mean = np.mean(roi[:, :, 1])
        v_mean = np.mean(roi[:, :, 2])

        print(f"绿色的 HSV 值：H={h_mean}, S={s_mean}, V={v_mean}")

    # 按下 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源并关闭窗口
cap.release()
cv2.destroyAllWindows()