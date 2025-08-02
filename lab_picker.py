import cv2
import numpy as np

def color_picker(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel = param['lab'][y,x]
        color_name = "未标记"
        if flags & cv2.EVENT_FLAG_CTRLKEY:
            color_name = "灰色"
            param['samples']['gray'].append(pixel)
        elif flags & cv2.EVENT_FLAG_SHIFTKEY:
            color_name = "黄色"
            param['samples']['yellow'].append(pixel)
            
        print(f"{color_name} LAB值: L={pixel[0]}, a={pixel[1]}, b={pixel[2]}")
        
        # 如果采样点足够多，计算范围
        for color in ['gray', 'yellow']:
            samples = param['samples'][color]
            if len(samples) >= 5:
                samples = np.array(samples)
                min_vals = np.min(samples, axis=0)
                max_vals = np.max(samples, axis=0)
                print(f"\n{color}颜色范围:")
                print(f"Lower: L={min_vals[0]-5}, a={min_vals[1]-5}, b={min_vals[2]-5}")
                print(f"Upper: L={max_vals[0]+5}, a={max_vals[1]+5}, b={max_vals[2]+5}\n")

def main():
    # 创建窗口
    cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
    cv2.namedWindow('LAB Channels', cv2.WINDOW_NORMAL)
    
    # 初始化采样数据
    samples = {
        'yellow': [],
        'gray': []
    }
    
    # 创建图像处理参数
    param = {
        'lab': None,
        'samples': samples
    }
    
    # 设置鼠标回调
    cv2.setMouseCallback('Original', color_picker, param)
    
    cap = cv2.VideoCapture(0)
    print("\n使用说明:")
    print("1. 按住Ctrl点击采集灰色样本")
    print("2. 按住Shift点击采集黄色样本")
    print("3. 每个颜色采集5个以上的点会自动计算范围")
    print("4. 按ESC退出\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # 转换到LAB空间
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        param['lab'] = lab
        
        # 分离LAB通道
        l_channel, a_channel, b_channel = cv2.split(lab)
        
        # 显示原始图像
        cv2.imshow('Original', frame)
        
        # 创建LAB通道的可视化
        lab_display = np.vstack([
            cv2.equalizeHist(l_channel),  # L通道
            cv2.equalizeHist(a_channel),  # a通道
            cv2.equalizeHist(b_channel)   # b通道
        ])
        cv2.imshow('LAB Channels', lab_display)
        
        # 添加标签
        x, y = 10, 30
        labels = ['L Channel', 'a Channel', 'b Channel']
        for i, label in enumerate(labels):
            y_pos = y + i * l_channel.shape[0]
            cv2.putText(lab_display, label, (x, y_pos),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        
        if cv2.waitKey(1) & 0xFF == 27:
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
