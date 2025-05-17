import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point

class XYPoseNode(Node):
    def __init__(self):
        super().__init__('xy_pose_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/phantom/pose',
            self.pose_callback,
            10)
        self.publisher = self.create_publisher(Point, '/phantom/xy_pose', 10)
        self.get_logger().info('XY Pose Node started')

    def pose_callback(self, msg):
        xy_point = Point()
        xy_point.x = msg.pose.position.x
        xy_point.y = msg.pose.position.y
        self.publisher.publish(xy_point)
        self.get_logger().info(f'Published XY: ({xy_point.x}, {xy_point.y})')

def main(args=None):
    rclpy.init(args=args)
    node = XYPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

