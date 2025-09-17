#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf_transformations

class GoalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        timer_period = 20.0  # 每秒发布一次
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PoseStamped()
        # 设置消息头和时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # 确保与你的TF树中的坐标系一致
        # 设置目标位置 (x, y, z)
        msg.pose.position.x = 0.5
        msg.pose.position.y = 0.1
        msg.pose.position.z = 0.0
        # 设置目标朝向 (四元数)。这里以朝向偏航角0度（正前方）为例
        q = tf_transformations.quaternion_from_euler(0, 0, 0)  # roll, pitch, yaw (弧度)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing goal pose: x={msg.pose.position.x}, y={msg.pose.position.y}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()