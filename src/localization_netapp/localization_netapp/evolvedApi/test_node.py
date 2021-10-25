from rclpy.node import Node
from .simulator import SimulatorAPI
from std_msgs.msg import String


class TestNode(Node):
    """
    This class is a ros node for testing the interaction with the NEF
    simulator api
    """

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sapi = SimulatorAPI()

    def timer_callback(self):
        msg = String()
        cell_id = self.sapi.read_cellid('202010000000003')
        msg.data = str(cell_id)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: CellID "%s"' % msg.data)