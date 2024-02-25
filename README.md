# 2023_Winter_Camp_Autodriving_SW
2023학년도 동계(겨울)방학 단기 교육 비교과 프로그램: ROS2 기반 자율주행 SW종합설계

```
cd
source /opt/ros/humble/setup.bash
git clone https://github.com/SKKU-AutoLab-VSW/2023_Winter_Camp_Autodriving_SW ros2_ws

cd ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install

source ./install/local_setup.bash
```
