

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from turtlebot3_msgs.srv import Dqn


class DQNEnvironment(Node):
    def __init__(self):
        super().__init__('dqn_environment')

        """**************************************
        Initialize variables
        **************************************"""
        self.goal_pose_x            = 0.0
        self.goal_pose_y            = 0.0
        self.last_pose_x            = 0.0
        self.last_pose_y            = 0.0
        self.last_pose_theta        = 0.0
        
        self.action_size            = 5
        self.done                   = False
        self.fail                   = False
        self.succeed                = False

        self.goal_angle             = 0.0
        self.goal_distance          = 1.0
        self.init_goal_distance     = 1.0
        self.scan_ranges            = []
        self.min_obstacle_distance  = 10.0
        self.min_obstacle_angle     = 10.0

        self.local_step             = 0

        """**************************************
        Initialize ROS publishers and subscribers
        **************************************"""
        qos = QoSProfile(depth=10)

        # initialize publishers
        self.cmd_vel_pub            = self.create_publisher(Twist, 'cmd_vel', qos)
        
        # initilize subscribers
        self.goal_pose_sub          = self.create_subscription(
            Pose,
            'goal_pose',
            self.goal_pose_callback,
            qos)
        self.odom_sub               = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)
        self.scan_sub               = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        # initialize client
        self.task_succeed_client    = self.create_client(Empty, 'task_succeed')
        self.task_fail_client       = self.create_client(Empty, 'task_file')

        # initialize servers
        self.dqn_com_server         = self.create_service(Dqn, 'dqn_com', self.dqn_com_callback)
    
    """**************************************
    Callback functions and relevant functions
    **************************************"""
    def goal_pose_callback(self, msg):
        self.goal_pose_x = msg.position.x
        self.goal_pose_y = msg.position.y
    
    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        goal_distance = math.sqrt(
            (self.goal_pose_x-self.last_pose_x)**2
            + (self.goal_pose_y-self.last_pose_y)**2)
        
        path_theta = math.atan2(
            self.goal_pose_y - self.last_pose_y,
            self.goal_pose_x - self.last_pose_x)
        
        goal_angle = path_theta - self.last_pose_theta
        if goal_angle > math.pi:
            goal_angle -= 2*math.pi
        elif goal_angle < -math.pi:
            goal_angle += 2*math.pi
        
        self.goal_distance = goal_distance
        self.goal_angle = goal_angle
    
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.min_obstacle_distance = min(self.scan_ranges)
        self.min_obstacle_angle = np.argmin(self.scan_ranges)
    
    def get_state(self):
        state = list()
        state.append(float(self.goal_distance))
        state.append(float(self.goal_angle))
        state.append(float(self.min_obstacle_distance))
        state.append(float(self.min_obstacle_angle))
        self.local_step += 1

        # succeed
        if self.goal_distance < 0.05:           # unit: m
            print('Goal! :)')
            self.succeed = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())   # robot stop
            self.local_step = 0
            req = Empty.Request()
            while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('[environment 125] service not available, waiting again...')
            self.task_succeed_client.call_async(req)
        
        if self.local_step == 500:
            print('Time out! :(')
            self.done = True
            self.local_step = 0
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('[environment 134] service not availbale, waiting again...')
            self.task_fail_client.call_async(req)
        
        return state
    
    def reset(self):
        return self.state
    
    def dqn_com_callback(self, request, response):
        action =request.action
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = ((self.action_size-1)/2 - action) * 1.5
        self.cmd_vel_pub.publish(twist)

        response.state = self.get_state()
        response.reward = self.get_reward(action)
        response.done = self.done

        if self.done:
            self.done = False
            self.succeed = False
            self.fail = False
        
        if request.init:
            self.init_goal_distance = math.sqrt(
                (self.goal_pose_x-self.last_pose_x)**2
                + (self.goal_pose_y - self.last_pose_y)**2)
        
        return response
    
    def get_reward(self, action):
        yaw_reward = 1 - 2*math.sqrt(math.fabs(self.goal_angle/math.pi))
        distance_reward = (2*self.init_goal_distance) / \
            (self.init_goal_distance + self.goal_distance) - 1
        
        # reward for avoiding obstacles
        if self.min_obstacle_distance < 0.25:
            obstacle_reward = -2
        else:
            obstacle_reward = 0
        
        reward = yaw_reward + distance_reward + obstacle_reward

        # + for succeed, - for fail
        if self.succeed:
            reward += 5
        elif self.fail:
            reward -= 10
        print(reward)

        return reward

    """**************************************
    Below should be replaced when porting for
    ROS2 Python tf_conversions is done
    **************************************"""
    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    

def main(args=None):
    rclpy.init(args=args)
    dqn_environment = DQNEnvironment()
    rclpy.spin(dqn_environment)

    dqn_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()