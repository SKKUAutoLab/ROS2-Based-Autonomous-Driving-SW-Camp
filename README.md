# 2023학년도 동계(겨울)방학 단기 교육 비교과 프로그램: 
## ROS2 기반 자율주행 SW종합설계

### Initial Setting

#### Installation
```
sudo apt update
sudo apt upgrade -y
sudo apt install git
sudo apt install python-pip -y
pip install setuptools==58.2.0
sudo pip install pyserial
sudo pip install keyboard
pip install opencv-python
pip install ultralytics
```
#### Clone
```
cd
git clone https://github.com/SKKU-AutoLab-VSW/2023_Winter_Camp_Autodriving_SW ros2_ws
cd ros2_ws
```

#### Build
```
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
```


### ROS2 Node Execution

#### The commands to be executed before running ROS2 nodes.
팀번호 수정하고 Enter
```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=팀번호
```

#### Node Execution Commands
```
ros2 run load_img cam_pub
```
```
ros2 run yolov8_ros predict
```
```
ros2 run img_process img_post
```
```
ros2 run img_process img_grad
```
```
ros2 run control_motor gen_control_data
```
```
ros2 run control_motor convert_protocol
```
```
ros2 run control_motor send_serial 
```



