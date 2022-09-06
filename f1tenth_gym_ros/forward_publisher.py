import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ForwardPublisher(Node):

    def __init__(self):
        super().__init__("going_forward")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel",10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)



    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    forward_publisher = ForwardPublisher()

    rclpy.spin(forward_publisher)

    forward_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()