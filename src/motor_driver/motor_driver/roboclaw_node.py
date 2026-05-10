import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from roboclaw_python.roboclaw_3 import Roboclaw

class RoboClawNode(Node):
    def __init__(self):
        super().__init__('roboclaw_node')
        self.timer = self.create_timer(1,self.read_encoders)

        # Motor Speed Limits
        self.motorHigh = 127
        self.motorLow = 0
        self.inches_to_meters = 0.0254
        self.wheel_circumference = 13*math.pi*self.inches_to_meters
        self.counts = 512


        # Initializing Roboclaw
        self.roboclaw = Roboclaw("/dev/ttyACM0", 38400)
        self.address = 0x80
        self.roboclaw.Open()
        self.roboclaw.SpeedM1(self.address,0)
        self.roboclaw.SpeedM2(self.address,0)

        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,             # message type
            '/cmd_vel',        # topic name
            self.cmd_vel_callback,  # callback function
            10                 # QoS queue size
        )

        self.get_logger().info("Starting Roboclaw driver node.... ")
    
    def read_encoders(self):
        self.get_logger().info(f"M1 Speed: {self.roboclaw.ReadEncM1(self.address)}")
        self.get_logger().info(f"M2 Speed: {self.roboclaw.ReadEncM2(self.address)}")

    def cmd_vel_callback(self, msg):
        linear = int(msg.linear.x)
        counts_per_meter = (self.counts / self.wheel_circumference)
        linear = round(linear * counts_per_meter) 
        angular = msg.angular.z
  
        self.get_logger().info("Moving")
        self.roboclaw.SpeedM1(self.address,linear)
        self.roboclaw.SpeedM2(self.address,linear)
        self.get_logger().info(f'{linear} counts/s')
        self.get_logger().info(f'M1 Speed: {self.roboclaw.ReadISpeedM1(self.address)}')
        self.get_logger().info(f'M2 Speed: {self.roboclaw.ReadISpeedM2(self.address)}')
        # self.roboclaw.ForwardM1(self.address,linear)
        # self.roboclaw.ForwardM2(self.address,linear)
        time.sleep(2)
        self.roboclaw.SpeedM1(self.address,0)
        self.roboclaw.SpeedM2(self.address,0)
        # self.roboclaw.ForwardM1(self.address,0)
        # self.roboclaw.ForwardM2(self.address,0)
        
def main(args=None):
    rclpy.init(args=args)
    node = RoboClawNode()
    rclpy.spin(node)
    rclpy.shutdown()

if '__name__' == '__main__':
    main()