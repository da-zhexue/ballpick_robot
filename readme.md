把整个文件夹放在ros2_ws下，把该文件夹名字改为src，然后在ros2_ws下打开终端，输入以下命令：  
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



在cam下，get_cam_matrix.py是用来获取相机内参和畸变系数的; cam.py是用来计算坐标的。  
  
相机标定：
    
1. 准备标定图像  
  
    打印一个棋盘格标定板（可以在网上找到或使用OpenCV示例中的）  
  
    将棋盘格固定在一个平坦的表面上  
  
    使用您的USB相机从不同角度和距离拍摄至少10-20张棋盘格照片  
  
    将这些照片保存在一个文件夹中（例如 calibration_images）  
  
2. 设置参数  
  
在get_cam_matrix.py代码中修改以下参数以适应您的设置：  
  
    images_path: 标定图像的路径和格式  
  
    chessboard_size: 棋盘格的内角点数量（通常比实际方格数少1）  
  
    square_size: 每个棋盘格方格的实际物理尺寸（单位：米）  
  
    calibration_file: 保存标定结果的文件路径  

3. 运行标定  
  
运行代码后，它将：  
  
    读取所有标定图像  
  
    检测每张图像中的棋盘格角点  
  
    使用这些角点计算相机内参和畸变系数  
  
    保存标定结果到文件  
  
    显示标定前后的图像对比，以验证标定效果  
  
4. 使用标定结果  
  
标定完成后，您可以将得到的 camera_matrix 和 dist_coeffs 用于cam.py代码中。  
