import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class Subscriber(Node):

    def __init__(self):
        super().__init__('clicked_point_sub_node')
        self.subscriber_ = self.create_subscription(PointStamped, 'clicked_point', self.callback, 1)
        self.subscriber_  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f' % (msg.point.x, msg.point.y, msg.point.z))

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin_once(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()