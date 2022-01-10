import rclpy
from rclpy.node import Node

from vertiport_system_msg.msg import VertiportLanding, VertiportTakeoff
from geometry_msgs.msg import Point


class Request(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.takeoff_publisher_ = self.create_publisher(VertiportTakeoff, 'request_takeoff', 10)

        self.landing_publisher_ =self.create_publisher(VertiportLanding, 'request_landing', 10)

        
        self.publish_takeoff()
        #self.publish_landing()
        


    def publish_takeoff(self):
        location = Point()
        location.x = 50.0
        location.y = 100.0
        location.z = 200.0
        msg = VertiportTakeoff()
        msg.destination = location
        
        self.takeoff_publisher_.publish(msg)
        self.get_logger().info("Pubishing takeoff!!")

    def publish_landing(self):
        location = Point()
        location.x = 150.0
        location.y = 300.0
        location.z = -100.0
        msg = VertiportLanding()
        msg.current_location = location
        
        msg.vehicle_id = A123
        self.landing_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Request()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
