from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from time import sleep

from threading import Thread


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

poses = dict()

class Robot(Node):
    def __init__(self):
        super().__init__('navigate_robot')
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
        self.rest = True

    
    def send_goal(self, table):
        self.rest = False
        self.cur_goal = table
        # self.get_logger().info('fdhsdkjhfjksdhfgjdhsg')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose = self.poses[table]
        # self._navigate_action_client.wait_for_server()
        self._navigate_action_client.wait_for_server()
        self._send_goal_future = self._navigate_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        # print('sending goal')
        # self._send_goal_future = self._navigate.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        # print('result')
        # self._send_goal_future.add_done_callback(self.goal_response_callback)
        # print('finish')
    
    def goal_response_callback(self, future):
        print('goal_response_callback')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info(f'Goal accepted --> {self.cur_goal}')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        print('get_result_callback')
        self.get_logger().info(f'Goal achieved --> {self.cur_goal}')
        if self.cur_goal != 'Start Point':
            sleep(5)
            self.send_goal(self.poses['Start Point'])
        self.rest = True
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        print('feedback_callback')
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining * 100:.2f} cm')
        sleep(1)
    
    def _info(self):
        while True:
            print(f'is rest:      {self.rest}')
            print(f'current_goal: {self.cur_goal}')
            print('-------------------------------')
            sleep(1)


def start(robot):
    while True:
        if robot.rest:
            robot.send_goal(input('Table # to deliver: '))


def main(args=None):
    print('dsadas')
    rclpy.init(args=args)
    print(1)
    robot = Robot()
    # info_thread = Thread(target=robot._info)
    # start_thread = Thread(target=start, args=(robot,))
    # start_thread.start()
    # # info_thread.start()
    start(robot)
    rclpy.spin(robot)


if __name__ == '__main__':
    main()

