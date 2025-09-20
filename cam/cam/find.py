#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool 
import torch
import cv2
import numpy as np

class BallDetectNode(Node):
    def __init__(self):
        super().__init__('ball_detect_node')
        
        # 示例相机参数 (需要替换为实际标定结果)
        camera_matrix = np.array([[500, 0, 320],
                                [0, 500, 240],
                                [0, 0, 1]], dtype=np.float32)
        
        dist_coeffs = np.array([-0.1, 0.01, 0, 0, 0], dtype=np.float32)  # 示例畸变系数
        
        # 相机安装参数 (需要替换为实际测量值)
        camera_height = 0.5  # 相机高度0.5米
        camera_pitch = np.radians(30)  # 相机向下倾斜30度
        self.detected_point = np.array([320, 300]) # 目标中心位置

        self.localizer = ItemLocalizer(camera_matrix, dist_coeffs, camera_height, camera_pitch)

        self.coord_publisher = self.create_publisher(PointStamped, 'ball_center_coords', 10)
        self.target_pose_reached = False # 目标位姿是否已到达
        self.target_pose_reached_subscriber = self.create_subscription(Bool, 'target_pose_reached', self.target_pose_reached_callback, 10)

        self.get_logger().info("正在加载YOLOv5模型...")
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True, force_reload=False, trust_repo=True)
        self.get_logger().info("YOLOv5模型加载成功")
        self.model.conf = 0.5  # 设置置信度阈值
        self.ball_class_id = 32 
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开相机")
            return
        
        self.timer = self.create_timer(0.1, self.process_frame)  # 10Hz
        self.get_logger().info("球类检测节点已启动，开始发布球体中心坐标...")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("无法获取相机帧")
            return
        
        results = self.model(frame)
        detections = results.xyxy[0].numpy()  # 格式为[x1, y1, x2, y2, confidence, class]
        for det in detections:
            if len(det) < 6:
                continue
            x1, y1, x2, y2, conf, cls_id = det

            # if int(cls_id) != self.ball_class_id:
            #     continue
            if conf < self.model.conf:
                continue

            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            self.detected_point = np.array([center_x, center_y])
            item_position = self.localizer.calculate_item_position(self.detected_point)
            # print(f"物品在小车坐标系中的位置: X={item_position[0]:.2f}m, Y={item_position[1]:.2f}m, Z={item_position[2]:.2f}m")

            ball_width = x2 - x1
            ball_height = y2 - y1

            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = "camera_frame"
            point_msg.point.x = float(item_position[0])
            point_msg.point.y = float(item_position[1])
            point_msg.point.z = 0.0  # 2D图像，z坐标为0
            if self.target_pose_reached:
                self.coord_publisher.publish(point_msg)
                self.get_logger().info(f"已到达目标位姿，发布球体中心坐标: ({item_position[0]:.2f}, {item_position[1]:.2f})")
            
            class_name = self.model.names[int(cls_id)]
            self.get_logger().info(f"检测到{class_name}中心坐标: ({item_position[0]:.2f}, {item_position[1]:.2f}), "
                                  f"尺寸: {ball_width:.1f}x{ball_height:.1f}, "
                                  f"置信度: {conf:.2f}")
            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(frame, (int(x1), int(y1)-25), (int(x1)+text_size[0], int(y1)), (0, 255, 0), -1)
            cv2.putText(frame, label, (int(x1), int(y1)-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
            size_text = f"W:{ball_width:.1f} H:{ball_height:.1f}"
            cv2.putText(frame, size_text, (int(x1), int(y2)+20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow('Ball Detection', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

    def target_pose_reached_callback(self, msg):
        self.target_pose_reached = msg.data 

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

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()