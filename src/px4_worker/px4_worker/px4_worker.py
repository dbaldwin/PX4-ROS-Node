import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus
from std_msgs.msg import String


class PX4Demo(Node):

    def __init__(self):
        super().__init__(node_name='px4_demo') #type: ignore
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscriber = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
    #     self.publisher_ = self.create_publisher(String, 'topic', 10)
    #     timer_period = 0.5  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)
    #     self.i = 0

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

    def vehicle_status_callback(self, msg):
        print("lol")
        self.get_logger().info("GOT A MESSAGE")

def main(args=None):
    rclpy.init(args=args)
    try:
        px4_demo = PX4Demo()
        rclpy.spin(px4_demo)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        px4_demo.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

if __name__ == '__main__':
    main()
