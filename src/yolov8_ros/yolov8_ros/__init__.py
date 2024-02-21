import os
from datetime import datetime

def get_path(file_name=None):
    p = os.path.dirname(os.path.abspath(__file__)).split("/")
    LIB_PATH = os.path.join("/", *p[1:4], "src", *p[5:6], *p[5:6], "lib", file_name)
    return LIB_PATH

def get_time(is_img=True):
    now = datetime.now()
    now = now.strftime('%y%m%d_%H%M%S')
    if is_img:
        result = now + '.png' 
    return result