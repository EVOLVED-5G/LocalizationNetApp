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
        utils.get_configs()

        # Create ROS publisher
        publisher_topic_name_user_1 = "cell_id_" + utils.ue_external_id_1[0:5]
        publisher_topic_name_user_2 = "cell_id_" + utils.ue_external_id_2[0:5]

        self.publisher_1_ = self.create_publisher(
            Int32, publisher_topic_name_user_1, 10
        )
        self.publisher_2_ = self.create_publisher(
            Int32, publisher_topic_name_user_2, 10
        )
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Create a subscription, that will notify us 1000 times, for the next 1 day starting from now
        expire_time = (
            datetime.datetime.utcnow() + datetime.timedelta(days=1)
        ).isoformat() + "Z"

        host = utils.get_host_of_the_nef_emulator()
        token = utils.get_token()
        self.location_subscriber = LocationSubscriber(
            host,
            utils.certificates_folder,
            utils.capif_host,
            utils.capif_https_port,
        )
        self.get_logger().info("Authentication Done")

        # Create subscription for first external id
        self.subscription_1 = self.location_subscriber.create_subscription(
            netapp_id=utils.netapp_id,
            external_id=utils.ue_external_id_1,
            notification_destination=utils.get_host_of_the_netapp(),
            maximum_number_of_reports=1000,
            monitor_expire_time=expire_time,
        )
        self.get_logger().info(
            "Successfully subscribed to location monitoring for %s"
            % utils.ue_external_id_1
        )
        print(self.subscription_1)

        # Create subscription for second external id
        self.subscription_2 = self.location_subscriber.create_subscription(
            netapp_id=utils.netapp_id,
            external_id=utils.ue_external_id_2,
            notification_destination=utils.get_host_of_the_netapp(),
            maximum_number_of_reports=1000,
            monitor_expire_time=expire_time,
        )
        self.get_logger().info(
            "Successfully subscribed to location monitoring for %s"
            % utils.ue_external_id_2
        )
        print(self.subscription_2)

    def destroy_node(self):
        id_1 = self.subscription_1.link.split("/")[-1]
        id_2 = self.subscription_2.link.split("/")[-1]
        self.location_subscriber.delete_subscription(self.netapp_id, id_1)
        self.location_subscriber.delete_subscription(self.netapp_id, id_2)
        return super().destroy_node()

    def timer_callback(self):
        msg_user_1 = Int32()
        cell_id_user_1 = netapp_utils.read_cellid_user_1()
        msg_user_1.data = cell_id_user_1
        self.publisher_1_.publish(msg_user_1)
        self.get_logger().debug('Publishing User 1: CellID "%s"' % msg_user_1.data)

        msg_user_2 = Int32()
        cell_id_user_2 = netapp_utils.read_cellid_user_2()
        msg_user_2.data = cell_id_user_2
        self.publisher_2_.publish(msg_user_2)
        self.get_logger().debug('Publishing User 2: CellID "%s"' % msg_user_2.data)
