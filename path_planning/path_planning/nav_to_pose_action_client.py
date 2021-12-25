from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')
        self._action_client = ActionClient(self, NavigateToPose, 'NavigateToPose')

    def send_goal(self):
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = 0.7866162657737732
        goal_pose.pose.pose.position.y = 1.8088958263397217
        goal_pose.pose.pose.position.z = 0.002471923828125

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    action_client.send_goal()

    rclpy.spin_once(action_client)


if __name__ == '__main__':
    main()