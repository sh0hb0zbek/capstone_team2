

import os
import random
import sys

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty

class DQNGazebo(Node):
    def __init__(self):
        super().__init__('dqn_gazebo')

        """**************************************
        Initialize variables
        **************************************"""
        # entity 'goal'
        self.entity_dir_path            = os.path.dirname(os.path.realpath(__file__))
        self.entity_dir_path            = self.entity_dir_path.replace(
            'dqn/dqn',
            'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_dqn_world/goal_box')
        self.entity_path                = os.path.join(self.entity_dir_path, 'model.sdf')
        self.entity                     = open(self.entity_path, 'r').read()
        self.entity_name                = 'goal'

        self.goal_pose_x                = 0.5
        self.goal_pose_y                = 0.0

        self.init_state                 = False

        """**************************************
        Initialize ROS publishers, subscriberss 
        and clients
        **************************************"""
        qos                             = QoSProfile(depth=10)

        # initialize publishers
        self.goal_pose_pub              = self.create_publisher(Pose, 'goal_pose', qos)

        # initialize clients
        self.delete_entity_client       = self.create_client(DeleteEntity, 'delete_entity')
        self.spawn_entity_client        = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client    = self.create_client(Empty, 'reset_simulation')

        # initilize servers
        self.task_succeed_server        = self.create_service(Empty, 'task_succeed', self.task_succeed_callback)
        self.task_fail_server           = self.create_service(Empty, 'task_fail', self.task_fail_callback)

        # process
        self.publish_timer              = self.create_timer(0.01, self.publish_callback)
    
    """**************************************
    Callback functions and relevant functions
    **************************************"""
    def publish_callback(self):
        # init
        if not self.init_state:
            self.delete_entity()
            self.reset_simulation()
            self.init_state = True
            print('Goal pose: ', self.goal_pose_x, self.goal_pose_y)
        
        # publish goal pose
        goal_pose = Pose()
        goal_pose.position.x = self.goal_pose_x
        goal_pose.position.y = self.goal_pose_y
        self.goal_pose_pub.publish(goal_pose)
        self.spawn_entity()
    
    def task_succeed_callback(self, request, response):
        self.delete_entity()
        self.generate_goal_pose()
        self.print('generate a new goal :)')

        return response
    
    def task_fail_callback(self, request, response):
        self.delete_entity()
        self.reset_simulation()
        self.generate_goal_pose()
        print('reset the gazebo environment :(')
        
        return response
    
    def generate_goal(self):
        goal_pose_list = [
            [1.2, -3.0], [-0.5, -4.2], [2.0, -4.2], [-0.8, -3.0], [1.2, -3.0],
            [3.2, -3.5], [3.2,  -1.0], [3.2,  1.0], [1.2,  -0.5], [1.5,  1.2]]
        index = random.randrange(0, 10)
        self.goal_pose_x = goal_pose_list[index][0]
        self.goal_pose_y = goal_pose_list[index][1]
        print('Goal pose:', self.goal_pose_x, self.goal_pose_y)
    
    def reset_simulation(self):
        req = Empty.Request()
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[gazebo 100] service not available, waiting again...')
        self.reset_simulation_client.call_async(req)
    
    def delete_entity(self):
        req = DeleteEntity.Request()
        req.name = self.entity_name
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[gazebo 107] service not available, waiting again...')
        self.delete_entity_client.call_async(req)
    
    def spawn_entity(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_pose_x
        goal_pose.position.y = self.goal_pose_y
        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = self.entity
        req.initial_pose = goal_pose
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[gazebo 119] service not available, waiting again...')
        self.spawn_entity_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    dqn_gazebo = DQNGazebo()
    rclpy.spin(dqn_gazebo)

    dqn_gazebo.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()