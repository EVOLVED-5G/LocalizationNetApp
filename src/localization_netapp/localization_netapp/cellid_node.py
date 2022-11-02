from rclpy.node import Node
from evolved5g.sdk import LocationSubscriber
import datetime
import evolvedApi.simulator as simulator
from std_msgs.msg import Int32
import os


class CellidNode(Node):
    """
    This class is a ros node for testing the interaction with the NEF
    simulator api
    """

    def __init__(self):
        super().__init__("cellid_node")
        self.publisher_ = self.create_publisher(Int32, "cellID", 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # Create a subscription, that will notify us 1000 times, for the next 1 day starting from now
        expire_time = (
            datetime.datetime.utcnow() + datetime.timedelta(days=1)
        ).isoformat() + "Z"
        self.netapp_id = "LocalizationNetapp"
        host = simulator.get_host_of_the_nef_emulator()
        token = simulator.get_token()
        self.location_subscriber = LocationSubscriber(host, token.access_token)
        self.get_logger().info("Authentication Done!!")

        external_id = os.environ.get("UE_EXTERNAL_ID")

        # In this example we are running flask at http://localhost:5000 with a POST route to (/monitoring/callback) in order to retrieve notifications.
        # If you are running on the NEF emulator, you need to provide a notification_destination with an IP that the
        # NEF emulator docker can understand
        # For latest versions of docker this should be: http://host.docker.internal:5000/monitoring/callback"
        # Alternative you can find the ip of the HOST by running 'ip addr show | grep "\binet\b.*\bdocker0\b" | awk '{print $2}' | cut -d '/' -f 1'
        # See article for details: https://stackoverflow.com/questions/48546124/what-is-linux-equivalent-of-host-docker-internal/61001152

        self.subscription = self.location_subscriber.create_subscription(
            netapp_id=self.netapp_id,
            external_id=external_id,
            notification_destination="http://host.docker.internal:8000/monitoring/callback",
            maximum_number_of_reports=1000,
            monitor_expire_time=expire_time,
        )

        # From now on we should retrieve POST notifications to http://host.docker.internal:8000/monitoring/callback

        print(self.subscription)

    def destroy_node(self):
        id = self.subscription.link.split("/")[-1]
        self.location_subscriber.delete_subscription(self.netapp_id, id)
        return super().destroy_node()

    def timer_callback(self):
        msg = Int32()
        cell_id = simulator.read_cellid()
        msg.data = cell_id
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: CellID "%s"' % msg.data)
