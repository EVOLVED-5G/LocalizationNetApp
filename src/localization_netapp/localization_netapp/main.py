
import rclpy
from .evolvedApi.test_node import TestNode

def main():
    rclpy.init(args=None)
    minimal_publisher = TestNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
