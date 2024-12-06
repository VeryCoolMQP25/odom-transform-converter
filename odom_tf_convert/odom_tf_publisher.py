import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.twist_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.broadcast_transform, 
            1
        )
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def broadcast_transform(self, msg: Odometry):
        # Create a TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        # Set translation
        transform.transform.translation = msg.pose.pose.position

        # Set rotation
        transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

