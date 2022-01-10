import rclpy
from rclpy.node import Node
from .vertiport import vertiport
import matplotlib.pyplot as plt
#import numpy as np
from numpy.core.function_base import linspace

from vertiport_system_msg.msg import VertiportLanding, VertiportTakeoff

vertiport_parameters = {
    "dimentions":[2,4,3],
    "drone_max_capacity": 6,
    "starting_drones": 4,
    "waypoint_generation_method": "Experimental",

    "landingpad_location_generation": "Experimental",
    "drone_separation_distance(cm)": 25,
}

drone_parameters = {
    "speed_control":1,
    "dimentions": [50,50,50], #in cm
    "dt": 0.1,
}

class VertiportController(Node):
        def __init__(self):
            super().__init__('minimal_subscriber')
            self.vertiport = vertiport(vertiport_parameters)
            self.vertiport.generateEnvironment()
            self.vertiport.generateWaypoints()


            self.landing_subscription = self.create_subscription(
                VertiportLanding,
                'request_landing',
                self.landing_cb,
                10
            )

            self.takeoff_subscription = self.create_subscription(
                VertiportTakeoff,
                'request_takeoff',
                self.takeoff_cb,
                10
            )

        def takeoff_cb(self,msg):
            takeoff_data ={
                "location":msg.destination,
            }
            print(takeoff_data)
            self.vertiport.takeoff(takeoff_data)
            self.get_logger().info("Pubishing takeoff!!")

            
            

        def landing_cb(self,msg):
            landing_data = {
                "location": [msg.curr_x, msg.curr_y, msg.curr_z],
                "vehicleid": msg.vehicle_id   
            }
            self.vertiport.landing(landing_data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = VertiportController()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
