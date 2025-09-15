import numpy as np
import cv2

class ItemLocalizer:
    def __init__(self, camera_matrix, dist_coeffs, camera_height, camera_pitch):
        """
        初始化物品定位器
        
        参数:
        - camera_matrix: 相机内参矩阵 (3x3)
        - dist_coeffs: 相机畸变系数 (1x5)
        - camera_height: 相机离地面的高度 (单位: 米)
        - camera_pitch: 相机俯仰角 (向下倾斜为正, 单位: 弧度)
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.camera_height = camera_height
        self.camera_pitch = camera_pitch
        
        # 提取内参
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]
        
        # 计算相机相对于小车的旋转矩阵 (绕X轴旋转pitch角度)
        self.R = np.array([[1, 0, 0],
                          [0, np.cos(camera_pitch), -np.sin(camera_pitch)],
                          [0, np.sin(camera_pitch), np.cos(camera_pitch)]])
        
        # 相机在小车坐标系中的位置 (假设相机在小车正上方)
        self.T = np.array([0, 0, camera_height])
    
    def undistort_point(self, point):
        """
        校正图像点的畸变
        
        参数:
        - point: 图像中的点 (x, y)
        
        返回:
        - 校正后的点 (x, y)
        """
        points = np.array([[[point[0], point[1]]]], dtype=np.float32)
        undistorted_points = cv2.undistortPoints(points, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
        return undistorted_points[0][0]
    
    def calculate_item_position(self, image_point):
        """
        计算物品在小车坐标系中的位置
        
        参数:
        - image_point: 图像中物品底部的像素坐标 (x, y)
        
        返回:
        - 物品在小车坐标系中的位置 (x, y, z)，z=0表示在地面上
        """
        # 1. 校正畸变
        undistorted_point = self.undistort_point(image_point)
        u, v = undistorted_point
        
        # 2. 转换到相机坐标系下的归一化方向向量
        x_cam = (u - self.cx) / self.fx
        y_cam = (v - self.cy) / self.fy
        z_cam = 1.0
        direction_cam = np.array([x_cam, y_cam, z_cam])
        
        # 3. 考虑相机姿态，转换到小车坐标系下的方向向量
        direction_vehicle = self.R @ direction_cam
        
        # 4. 计算射线与地面的交点
        # 射线方程: P = T + t * direction_vehicle
        # 我们需要找到t使得P_z = 0
        t = -self.T[2] / direction_vehicle[2]
        
        # 5. 计算交点坐标
        item_position = self.T + t * direction_vehicle
        
        return item_position
    
    def draw_coordinates(self, image, image_point, item_position):
        """
        在图像上绘制坐标信息 (用于调试和可视化)
        """
        # 绘制检测点
        cv2.circle(image, tuple(image_point.astype(int)), 5, (0, 0, 255), -1)
        
        # 添加坐标文本
        text = f"X: {item_position[0]:.2f}m, Y: {item_position[1]:.2f}m"
        cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return image

# 示例使用代码
if __name__ == "__main__":
    # 示例相机参数 (需要替换为实际标定结果)
    camera_matrix = np.array([[500, 0, 320],
                              [0, 500, 240],
                              [0, 0, 1]], dtype=np.float32)
    
    dist_coeffs = np.array([-0.1, 0.01, 0, 0, 0], dtype=np.float32)  # 示例畸变系数
    
    # 相机安装参数 (需要替换为实际测量值)
    camera_height = 0.5  # 相机高度0.5米
    camera_pitch = np.radians(30)  # 相机向下倾斜30度
    
    # 创建定位器
    localizer = ItemLocalizer(camera_matrix, dist_coeffs, camera_height, camera_pitch)
    
    # 模拟从图像中检测到的物品底部坐标
    # 这里假设使用某种目标检测算法获取了这个点
    detected_point = np.array([320, 300])  # 图像中的像素坐标
    
    # 计算物品位置
    item_position = localizer.calculate_item_position(detected_point)
    print(f"物品在小车坐标系中的位置: X={item_position[0]:.2f}m, Y={item_position[1]:.2f}m, Z={item_position[2]:.2f}m")
    
    # 以下部分可以集成到您的主程序中
    # 假设您已经捕获了一帧图像
    # frame = capture_frame_from_camera()
    # detected_point = detect_item(frame)  # 使用目标检测算法
    # item_position = localizer.calculate_item_position(detected_point)
    # frame_with_info = localizer.draw_coordinates(frame, detected_point, item_position)
    # cv2.imshow("Localization", frame_with_info)
    # cv2.waitKey(1)