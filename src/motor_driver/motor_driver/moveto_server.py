import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from wheelchair_interfaces.action import MoveTo
from roboclaw_python.roboclaw_3 import Roboclaw
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np

class MoveToServerNode(Node):
    def __init__(self):
        super().__init__("moveto_server")
        self.moveto_server = ActionServer(self,MoveTo,"moveto",goal_callback=self.goal_callback,execute_callback=self.execute_callback,cancel_callback=self.cancel_callback, callback_group=ReentrantCallbackGroup())
        self.get_logger().info("MoveTo Server has started.")

    def stop_motors(self):
        self.roboclaw.SpeedM1M2(self.address, 0, 0)
    
    def goal_callback(self, goal_request: MoveTo.Goal):
        self.get_logger().info("Received Goal.")
        self.get_logger().info("Accepted Goal.")
        goal_request.desired_pos.x
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        self.stop_motors()
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        result = MoveTo.Result()
        feedback = MoveTo.Feedback()

        # Initializing Roboclaw
        self.roboclaw = Roboclaw("/dev/ttyACM1", 38400)
        self.address = 0x80
        self.roboclaw.Open()
        # Initializing Variables
        goal = goal_handle.request
        x = goal.desired_pos.x # meters
        y = goal.desired_pos.y # meters
        vel = goal.desired_velocity # m/s
        self.inches_to_meters = 0.0254
        self.wheel_circumference = 13*math.pi*self.inches_to_meters
        self.counts = 512
        self.expected_time = x/vel
        elaspedTime = 0.0

        # Encoder Variables
        deltaM1_counts = 0
        deltaM2_count = 0
        counts_per_meter = (self.counts / self.wheel_circumference)
        meters_per_count = (self.wheel_circumference / self.counts)
        m1_encoder_status, M1_encoder_start_count, _ = self.roboclaw.ReadEncM1(self.address)
        m2_encoder_status, M2_encoder_start_count, _ = self.roboclaw.ReadEncM2(self.address)
        delta_counts = int(round(x *counts_per_meter))
        M1_encoder_target_count =round( M1_encoder_start_count + delta_counts) 
        M2_encoder_target_count =round( M2_encoder_start_count + delta_counts)
        speed_counts = round(vel * counts_per_meter)
        M1_encoder_current_count = M1_encoder_start_count
        M2_encoder_current_count = M2_encoder_start_count
        M1_encoder_prev_count = M1_encoder_start_count
        M2_encoder_prev_count = M2_encoder_start_count
        tolerance = 5 # delta count tolerance

        # Stall Variables 
        stall_threshold = 0.5 #seconds
        stall_time = 0 #seconds
        stall_delta = 3 #counts

        # Waypoints
        steps = 10 
        waypoints_m1 = np.linspace(M1_encoder_start_count,M1_encoder_target_count,steps,dtype=int)
        waypoints_m2 = np.linspace(M2_encoder_start_count,M2_encoder_target_count,steps,dtype=int)
        
        # Pure Translation
        if (x and not y):
            
            # Log Current and Target Encoder Count
            self.get_logger().info(f'Current M1 Encoder Count:{M1_encoder_start_count}')
            self.get_logger().info(f'Target M1 Encoder Count:{M1_encoder_target_count}')
            self.get_logger().info(f'Current M2 Encoder Count:{M2_encoder_start_count}')
            self.get_logger().info(f'Target M2 Encoder Count:{M2_encoder_target_count}')
            self.get_logger().info(f'M1 Waypoints: {waypoints_m1}')
            self.get_logger().info(f'M2 Waypoints: {waypoints_m2}')

            # Begin Motion Control
            try:
                startTime = time.perf_counter()
                prevTime = startTime
                
                for num in range(1,len(waypoints_m1)):
                    self.roboclaw.SpeedDistanceM1M2(self.address,speed_counts,int(waypoints_m1[num]),speed_counts,int(waypoints_m2[num]),1)

                    while abs(waypoints_m1[num] - M1_encoder_current_count) > tolerance or abs(waypoints_m2[num]-M2_encoder_current_count) > tolerance :

                        # Update Position and Velocity
                        currentTime = time.perf_counter()
                        elaspedTime = currentTime - startTime
                        current_x_m = abs(M1_encoder_current_count- M1_encoder_start_count) * (self.wheel_circumference/self.counts)

                        # current_x_velocity = current_x_m / (currentTime - prevTime)
                        m1_vel_status,current_m1_velocity,_ = self.roboclaw.ReadSpeedM1(self.address) 
                        m2_vel_status,current_m2_velocity,_ = self.roboclaw.ReadSpeedM2(self.address)

                        if m1_vel_status is None or m2_vel_status is None:
                            raise RuntimeError("Failed to read motor velocity.")

                        # Cancel Request Check
                        if (goal_handle.is_cancel_requested):
                            self.stop_motors()
                            goal_handle.canceled()
                            result.success = False
                            result.final_pos = float(current_x_m)
                            result.final_time = elaspedTime
                            return result
                        
                        # Encoder Check
                        M1status, M1_encoder_current_count, _ = self.roboclaw.ReadEncM1(self.address)
                        M2status, M2_encoder_current_count, _ = self.roboclaw.ReadEncM2(self.address)
                        if not (M1status or M2status):
                            self.stop_motors()
                            goal_handle.abort()
                            result.success = False
                            result.final_pos = float(current_x_m)
                            result.final_time = elaspedTime
                            return result
                        
                        # Timeout Check
                        if elaspedTime > self.expected_time:
                            self.stop_motors()
                            goal_handle.abort()
                            result.success = False
                            result.final_pos = float(current_x_m)
                            result.final_time = elaspedTime
                            return result
                        
                        # Stall Check
                        # if abs(M1_encoder_current_count - M1_encoder_prev_count) < stall_delta or abs(M2_encoder_current_count-M2_encoder_prev_count) < stall_delta:
                        #     stall_time += (currentTime - prevTime)

                        # if stall_time > stall_threshold:
                        #     self.stop_motors()
                        #     goal_handle.abort()
                        #     result.success = False
                        #     result.final_pos = float(current_x_m)
                        #     result.final_time = elaspedTime
                        #     return result
                        
                        # Update prev count and prev time
                        M1_encoder_prev_count= M1_encoder_current_count
                        M2_encoder_prev_count = M2_encoder_current_count
                        prevTime = currentTime

                        # Feedback Update
                        feedback.current_pos.x = float(current_x_m)
                        feedback.current_m1_velocity = float(current_m1_velocity)
                        feedback.current_m2_velocity = float(current_m2_velocity)
                        feedback.m1_encoder_count = int(M1_encoder_current_count)
                        feedback.m2_encoder_count = int(M2_encoder_current_count)
                        goal_handle.publish_feedback(feedback)
                        time.sleep(0.01)

                    # Waypoint Reached Log
                    self.get_logger().info(f"Waypoint {waypoints_m1[num]} reached for M1.")
                    self.get_logger().info(f"Waypoint {waypoints_m2[num]} reached for M2.")
                
                goal_handle.succeed()
                endTime = time.perf_counter()
                self.stop_motors()
                result.success = True
                result.final_pos.x = current_x_m
                result.final_time = endTime-startTime
                return result
            
            except Exception as e:
                self.stop_motors()
                goal_handle.abort()
                self.get_logger().info(f"An exception occured: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveToServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
