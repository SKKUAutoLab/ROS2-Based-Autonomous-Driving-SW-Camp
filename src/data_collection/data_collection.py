import string
import keyboard
import time
import serial
import cv2
import time
import uuid
import os
import sys
from datetime import datetime
import types
import marshal

'''
키 조작

w : 속도 +10
s : 속도 -10
a : 회전 -1     (- : 좌회전, + : 우회전)
d : 회전 +1

c: 카메라 프레임 캡쳐

r : 일시정지
f : 종료
'''

# TODO ----------------------------------------------------
# video 번호 입력
CAM_NUM = 2 

# 아두이노 포트번호 입력
PORT='/dev/ttyACM0' 

# 데이터 수집 경로 입력
DATA_COLLECTION_PATH='./load_img/load_img/lib/Collected_Datasets' 

# 가변저항 입력 핀
VARIBLE_RESISTER_INPUT_PIN = 'A2'

# 조향모터에 연결된 핀
STEERING_PIN1 = 7 
STEERING_PIN2 = 6

# 좌측 뒷바퀴에 연결된 핀
LEFT_REAR_PIN1 = 5 
LEFT_REAR_PIN2 = 4

# 우측 뒷바퀴에 연결된 핀
RIGHT_REAR_PIN1 = 3
RIGHT_REAR_PIN2 = 2

# 가변저항 최대 좌우측 값
VARIBLE_RESISTER_MOST_LEFT = 589 
VARIBLE_RESISTER_MOST_RIGHT = 481

# ----------------------------------------------------

pyc = open('data_collection_func.cpython-310.pyc', 'rb').read()
code = marshal.loads(pyc[16:])
module = types.ModuleType('module_name')
exec(code, module.__dict__)

def main():
    arduino_init_info = [VARIBLE_RESISTER_INPUT_PIN, STEERING_PIN1, STEERING_PIN2, LEFT_REAR_PIN1, LEFT_REAR_PIN2, RIGHT_REAR_PIN1, RIGHT_REAR_PIN2, VARIBLE_RESISTER_MOST_LEFT, VARIBLE_RESISTER_MOST_RIGHT]
    data_collector = module.Data_Collect(path=DATA_COLLECTION_PATH, cam_num=CAM_NUM, ser_port=PORT, arduino_info=arduino_init_info)
    
    try:
        data_collector.process()
    except KeyboardInterrupt:
        data_collector.interrupt_process()

    data_collector.ser.close()

    if data_collector.cap.isOpened():
        data_collector.cap.release()

    cv2.destroyAllWindows()
    sys.exit(0)

if __name__ == '__main__':
    main()
