from rclpy.node import Node
from .simulator import SimulatorAPI
from std_msgs.msg import Int32


class CellidNode(Node):
    """
    This class is a ros node for testing the interaction with the NEF
    simulator api
    """
    def __init__(self):
        super().__init__('cellid_node')
        self.publisher_ = self.create_publisher(Int32, 'cellID', 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.sapi = SimulatorAPI()
        self.get_logger().info('Autantication Done!!')
        json_response = self.sapi.location_subscription('10003@domain.com')
        self.get_logger().info('%s' % json_response)

    def timer_callback(self):
        msg = Int32()
        cell_id = self.sapi.read_cellid('202010000000001')
        msg.data = cell_id
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: CellID "%s"' % msg.data)
