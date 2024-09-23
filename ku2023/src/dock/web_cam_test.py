import cv2
import sys
from shape_detector.main import init_detector 
from shape_detector.main import detect_ellipse
init_detector()
cap = cv2.VideoCapture(0) # 클래스 생성
if not cap.isOpened():
    print("Camera open failed!") # 열리지 않았으면 문자열 출력
    sys.exit()

while True: # 무한 루프
    ret, frame = cap.read() # 두 개의 값을 반환하므로 두 변수 지정
    detect_ellipse("./find_model_20.pt", 224, frame)
    if not ret: # 새로운 프레임을 못받아 왔을 때 braek
        break
        
    # 정지화면에서 윤곽선을 추출
    

    cv2.imshow('frame', frame)


    # 10ms 기다리고 다음 프레임으로 전환, Esc누르면 while 강제 종료
    if cv2.waitKey(10) == 27:
        break