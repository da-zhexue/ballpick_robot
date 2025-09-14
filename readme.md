把整个文件夹放在ros2_ws下，把该文件夹名字改为src，然后在终端输入：  
```
colcon build
```
编译整个ros2_ws。  
  
然后在终端输入：  
```
source install/setup.bash
```
激活ros2环境。  
然后在终端输入：  
```
ros2 launch ballpick_robot robot_start.launch.py
```
启动小车。  
