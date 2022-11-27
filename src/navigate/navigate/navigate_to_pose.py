import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from time import sleep
import sys

from nav2_msgs.action import NavigateToPose

points = {
    'Start Point':  [[  0.0,   0.0], [0.0, 0.0,   0.0, 0.99]],
    '1':  [[ 1.74, -1.97], [0.0, 0.0,  0.99, -0.10]],
    '2':  [[ 1.74, -4.53], [0.0, 0.0,  0.99, -0.10]],
    '3':  [[ 2.99, -4.55], [0.0, 0.0,  0.65, -0.75]],
    '4':  [[ 2.24, -3.48], [0.0, 0.0,  0.94,  0.99]],
    '5':  [[ 3.01, -0.60], [0.0, 0.0, -0.61,  0.80]],
    '6':  [[ 4.11, -2.97], [0.0, 0.0, -0.11,  0.99]],
    '7':  [[ 4.08, -0.53], [0.0, 0.0,  0.11,  0.99]],
    '8':  [[ 4.32,  1.56], [0.0, 0.0,  0.99,  0.11]],
    '9':  [[ 2.32,  1.49], [0.0, 0.0,  0.99,  0.11]],
    '10': [[-1.71, -1.98], [0.0, 0.0,  0.12, -0.99]],
    '11': [[-2.21, -1.98], [0.0, 0.0, -0.99, -0.13]],
    '12': [[-1.71, -4.57], [0.0, 0.0,  0.12, -0.99]],
    '13': [[-2.21, -4.57], [0.0, 0.0, -0.99, -0.13]],
}

class NavigateActionCLient(Node):
    def __init__(self):
        super().__init__('navigate_action_client')
        self._navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.poses = dict()
        self.cur_goal = 'Start Point'
        for num, point in zip(points.keys(), points.values()):
            pose = Pose()
            pose.position.x    = point[0][0]
            pose.position.y    = point[0][1]
            pose.orientation.x = point[1][0]
            pose.orientation.y = point[1][1]
            pose.orientation.z = point[1][2]
            pose.orientation.w = point[1][3]
            self.poses[num]    = pose
        self.cur_goal = 'Start Point'

    def send_goal(self, table):
        self.cur_goal = table
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose = self.poses[table]

        self._navigate_action_client.wait_for_server()
        self._send_goal_future = self._navigate_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        print(f'Goal accepted --> {self.cur_goal}')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        if self.cur_goal != 'Start Point':
            print('Order is delivered Successfully')
        else:
            print('Come back to the Start Point')
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f'Distance remaining: {feedback.distance_remaining * 33:.2f} cm', end='\r')
        sleep(1)


def go_pose(args=None, table=None):
    rclpy.init(args=args)
    action_client = NavigateActionCLient()
    if table is None:
        table = input('Table # to deliver: ')
    action_client.send_goal(table=table)
    rclpy.spin(action_client)



def main(args=None):
    while True:
        go_pose(args)
        sleep(5)
        go_pose(args, 'Start Point')


if __name__ == '__main__':
    main()

    