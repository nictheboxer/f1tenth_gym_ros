# import Empty service module from std_srvs 
from std_srvs.srv import Empty
# import Twist msg module from geometry_msgs
from geometry_msgs.msg import Twist
# import rclpy ROS 2 client libraries
import rclpy
from rclpy.node import Node


class DriveServiceServer(Node):

    def __init__(self):

        # call the class constructor to initialize the node as drive_service_server, inherit from super class
        super().__init__('drive_service_server')
        # next create the service server object
        # create_service(service_type, name_of_the_service, service_callback)
        self.srv = self.create_service(Empty, 'driving', self.service_callback)
        # upon receiving the service request a publisher will publish to the cmd_vel topic to start driving 
        # the cmd_vel takes in Twist msg
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def service_callback(self, request, response):
        # This method is the service callback and it takes self, request and response as parameters
        # data are received through request
        # result is send through response

        # construct the Twist message
        msg = Twist()
        # send the car driving forward with velocity 0.5m/s - linear.x = 0.5
        msg.linear.x = 0.5
        # send the car driving straight - angular.z = 0.0
        msg.angular.z = 0.0
        # Publish the constructed message to the topic 
        self.publisher_.publish(msg)
        
        
        # finally return the response
        return response


def main(args=None):
    # initialize ROS 
    rclpy.init(args=args)
    # declare the DriveServiceServer 
    driving_service = DriveServiceServer()
    # start spinnig - spin() will wait for a request (i.e. ctrl+c) to kill the node
    rclpy.spin(driving_service)
    # shutdown all ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()