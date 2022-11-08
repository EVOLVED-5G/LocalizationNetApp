from rclpy.node import Node
from evolved5g.sdk import LocationSubscriber
import datetime
import evolvedApi.netapp_utils as netapp_utils
from std_msgs.msg import Int32
import os


class CellidNode(Node):
    """
    This class is a ros node for testing the interaction with the NEF
    simulator api
    """

    def __init__(self):
        super().__init__("cellid_node")

        # Initialize Utils class and get configs
        utils = netapp_utils.Utils()

        # Create ROS publisher
        publisher_topic_name = "cell_id_" + utils.ue_external_id[0:5]
        self.publisher_ = self.create_publisher(Int32, publisher_topic_name, 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Create a subscription, that will notify us 1000 times, for the next 1 day starting from now
        expire_time = (
            datetime.datetime.utcnow() + datetime.timedelta(days=1)
        ).isoformat() + "Z"
        
        host = utils.get_host_of_the_nef_emulator()
        token = utils.get_token()
        self.location_subscriber = LocationSubscriber(host, token.access_token, utils.certificates_folder, utils.capif_host, utils.capif_https_port)
        self.get_logger().info("Authentication Done")

        self.subscription = self.location_subscriber.create_subscription(
            netapp_id=utils.netapp_id,
            external_id=utils.ue_external_id,
            notification_destination=utils.get_host_of_the_netapp(),
            maximum_number_of_reports=1000,
            monitor_expire_time=expire_time,
        )
        self.get_logger().info("Successfully subscribed to location monitoring")
        print(self.subscription)

    def destroy_node(self):
        id = self.subscription.link.split("/")[-1]
        self.location_subscriber.delete_subscription(self.netapp_id, id)
        return super().destroy_node()

    def timer_callback(self):
        msg = Int32()
        cell_id = netapp_utils.read_cellid()
        msg.data = cell_id
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: CellID "%s"' % msg.data)
