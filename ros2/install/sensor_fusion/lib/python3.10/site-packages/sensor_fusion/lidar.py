import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader_node')
        self.get_logger().info('LiDAR Reader Node has been started.')

        # Create the subscriber for the LiDAR topic
        self.lidar_subscriber = self.create_subscription(
            PointCloud2,
            '/scan',  # Change this to your LiDAR topic
            self.lidar_callback,
            10
        )

        # Create the publisher for the new LiDAR topic
        self.lidar_publisher = self.create_publisher(
            PointCloud2,
            '/my_lidar_topic',
            10
        )

    def lidar_callback(self, msg):
        # Publish the received LiDAR message to the new topic
        self.get_logger().info('Received and publishing a LiDAR scan.')
        self.lidar_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReader()
    rclpy.spin(lidar_reader)
    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()