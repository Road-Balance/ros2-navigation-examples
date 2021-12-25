import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class Publisher(Node):

    def __init__(self):
        super().__init__('goal_pose_pub_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 1)
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = 2.2
        msg.pose.position.y = 0.0
        msg.pose.orientation.w = 1.0
        self.get_logger().info('Publishing  Goal Position  \n X= 2.2 \n Y=0.0 \n W = 1.0 ')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin_once(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()