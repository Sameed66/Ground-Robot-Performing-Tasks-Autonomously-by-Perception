import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageReader(Node):
    def __init__(self):
        super().__init__('image_reader_node')
        self.get_logger().info('Image Reader Node has been started.')

        # Create the subscriber for the image topic
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change this to your image topic
            self.image_callback,
            10
        )

        # Create the publisher for the new image topic
        self.image_publisher = self.create_publisher(
            Image,
            '/my_image_topic',
            10
        )

    def image_callback(self, msg):
        # Publish the received image message to the new topic
        self.get_logger().info('Received and publishing an image.')
        self.image_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    image_reader = ImageReader()
    rclpy.spin(image_reader)
    image_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()