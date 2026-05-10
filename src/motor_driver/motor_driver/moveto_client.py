import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from wheelchair_interfaces.action import MoveTo
from roboclaw_python.roboclaw_3 import Roboclaw

class MoveToClient(Node):
    def __init__(self):
        super().__init__("moveto_client")
        self.moveto_client = ActionClient(self,MoveTo,"moveto")
    
    def send_goal(self,desired_pos_x,desired_pos_y,desired_velocity):
        self.moveto_client.wait_for_server()
        goal = MoveTo.Goal()
        goal.desired_pos.x=desired_pos_x
        goal.desired_pos.y = desired_pos_y
        goal.desired_velocity = desired_velocity
        self.moveto_client.send_goal_async(goal,feedback_callback=self.goal_feedback_callback, )

    def cancel_goal(self):
        self.get_logger().info("Send a cancel goal request")
        self.goal_handle_.cancel_goal_async()

    def goal_response_callback(self, future):
        self.goal_handle: ClientGoalHandle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info("Goal was accepted.")
            self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal was rejected.")
    
    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.final_pos.x))
    
    def goal_feedback_callback(self, feedback_msg):
        current_x = feedback_msg.feedback.current_pos
        current_velocity = feedback_msg.feedback.current_velocity
        self.get_logger().info("Current X Pos: " + str(current_x) + " m")
        self.get_logger().info("Current Velocity: " + str(current_velocity) + " m/s")

def main(args=None):
    rclpy.init(args=args)
    node = MoveToClient()
    # node.send_goal(2,1)
    rclpy.spin(node)
    rclpy.shutdown()

if '__name__' == '__main__':
    main()