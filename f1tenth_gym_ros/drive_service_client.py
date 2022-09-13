# import the Empty service module from std_srvs
from std_srvs.srv import Empty
# import rclpy ROS 2 client libraries
import rclpy
from rclpy.node import Node


class DriveServiceClientAsync(Node):

    def __init__(self):

        # call the class constructor to initialize the node as drive_service_client, inherit from super class
        super().__init__('drive_service_client')
        # next create the service client object
        # create_client(service_type, name_of_the_service)
        self.client = self.create_client(Empty, 'driving')
        # safety check to check once per second whether the service server is awailable 
        while not self.client.wait_for_service(timeout_sec=1.0):
            # display warning message if service server not available 
            self.get_logger().info('driving service not available, waiting')
        
        # the empty request does not have any arguments, just create Empty.Request()
        self.request = Empty.Request()
        

    def send_request(self):
        
        #this function send the empty request
        self.future = self.client.call_async(self.request)


def main(args=None):
    # start the ROS comms 
    rclpy.init(args=args)
    # declare the DriveSeriveClient 
    client = DriveServiceClientAsync()
    # send the service request
    client.send_request()

    while rclpy.ok():
        # wait till the response is received or kill is sent (ctrl+c)
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # checks for the service server response and logs it
                response = client.future.result()
            except Exception as e:
                # Display response on screen
                client.get_logger().info(
                    'Service call not sucesfull %r' % (e,))
            else:
                # Display sucesfull response msg
                client.get_logger().info(
                    'the robot is driving' ) 
            break

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()