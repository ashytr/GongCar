import cv2

cap=cv2.VideoCapture(0)
# 初始化 QRCode 检测器
qrDecoder= cv2.QRCodeDetector()
cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
while (True):
    ret,img = cap.read()
    if not ret:
        print("No Camera")
    data, bbox, straight_qrcode = qrDecoder.detectAndDecode(img)
    #print(bbox)
    #print(data)
    #print(bbox[0])
    if bbox is not None and len(bbox) > 0 and len(bbox[0]) == 4:  
        points = bbox[0]
        top_left = (int(points[0][0]), int(points[0][1]))
        bottom_right = (int(points[2][0]), int(points[2][1]))
        #print(top_left)
        #print(bottom_right)
        cv2.rectangle(img, top_left, bottom_right, (0, 255, 0), 2)
        text_position = (top_left[0], top_left[1] - 10)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_color = (0, 0, 255) 
        thickness = 1
        cv2.putText(img, data, text_position, font, font_scale, font_color, thickness)
        print(data)
    cv2.imshow("Image with QR Code", img)
    cv2.waitKey(1)
