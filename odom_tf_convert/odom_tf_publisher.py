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
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("TF publisher started!")

    def broadcast_transform(self, msg: Odometry):
        self.broadcast_odom_to_base_link(msg)
        self.broadcast_base_link_to_laser()

    def broadcast_odom_to_base_link(self, msg: Odometry):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug("Published transform: odom -> base_link")

    def broadcast_base_link_to_laser(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser'

        transform.transform.translation.x = 0.14  
        transform.transform.translation.y = 0.0   
        transform.transform.translation.z = 1.12   

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug("Published transform: base_link -> laser")

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
