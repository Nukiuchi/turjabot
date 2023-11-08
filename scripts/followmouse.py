#!/usr/bin/env python3


import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

import pyautogui


class TrajectoryActionClient(Node):
    
    def __init__(self):

        super().__init__('points_publisher_node_action_client')
        self.topic = '/robot_cam_controller/joint_trajectory'
        self.joint_names = ['camera_pan_joint','camera_tilt_joint']
        
        self.max_pos = 0.5
        self.min_pos = -0.5
        
        self.trajectory_publisher = self.create_publisher(JointTrajectory,self.topic, 10)
        
        self.timer_period = 0.01  # seconds
        self.time = 0.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        
    def timer_callback(self):
        point = JointTrajectoryPoint()
        point.positions = self.get_mouse_pos()
        point.time_from_start = Duration(seconds=self.timer_period/2).to_msg()
        
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [ point ]
        
        self.trajectory_publisher.publish(msg)
        
        self.time = self.time + self.timer_period
    
    
    def get_mouse_pos(self):
        size = pyautogui.size()
        pos = pyautogui.position()
        
        mapped_x = self.num_to_range(pos[0], 0, size[0], self.min_pos, self.max_pos)
        mapped_y = self.num_to_range(pos[1], 0, size[1], self.min_pos, self.max_pos)
        
        #self.get_logger().info('Target position: ' + str(mapped_x) + ', ' + str(mapped_y))
        
        return [mapped_x, mapped_y]
        
        
    def num_to_range(self, num, inMin, inMax, outMin, outMax):
        return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax
                        - outMin))


def main(args=None):
    rclpy.init(args=args)
    
    action_client = TrajectoryActionClient()
    action_client.get_logger().info('\r\n\r\nStarting execution...\r\n\r\n')
    
    try:
        rclpy.spin(action_client)
    except Exception as error:
        action_client.get_logger().error('\r\n\rError occurred while executing:\r\n')
        action_client.get_logger().error(error)
        action_client.get_logger().info('\r\n\r\nStopping execution...\r\n\r\n')
        
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()