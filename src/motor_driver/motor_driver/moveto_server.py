import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.parameter import Parameter
from wheelchair_interfaces.action import MoveTo
from basicmicro import Basicmicro
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np

class MoveToServerNode(Node):
    def __init__(self):
        super().__init__("moveto_server")
        self.moveto_server = ActionServer(self,MoveTo,"moveto",goal_callback=self.goal_callback,execute_callback=self.execute_callback,cancel_callback=self.cancel_callback, callback_group=ReentrantCallbackGroup())
        self.add_post_set_parameters_callback(self.parameters_callback)
        self.get_logger().info("MoveTo Server has started.")

        # Declaring Parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("controller_address", 0x80)
        self.declare_parameter("frequency", 20)
        self.port = self.get_parameter("port").value
        self.address = self.get_parameter("controller_address").value
        self.frequency = self.get_parameter("frequency").value
        self.delay_time = 1/self.frequency

        # Initializing Controllers
        self.controller = Basicmicro(self.port, 38400, timeout=0.5)
        self.controller.Open()

    def parameters_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "port":
                self.port = param.value              
            elif param.name == "controller_address":
                self.address = param.value
            elif param.name == "frequency":
                self.frequency = param.value

    def stop_motors(self):
        self.controller.SpeedM1M2(self.address, 0, 0)
    
    def goal_callback(self, goal_request: MoveTo.Goal):
        self.get_logger().info("Received Goal.")
        self.get_logger().info("Accepted Goal.")
        goal_request.desired_pos.x
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        result = MoveTo.Result()
        feedback = MoveTo.Feedback()
        
        # Initializing Variables
        goal = goal_handle.request
        x = goal.desired_pos.x # meters
        y = goal.desired_pos.y # meters
        theta = goal.desired_pos.theta # radians
        vel = goal.desired_velocity # m/s
        ang_vel = goal.desired_angular_velocity # rad/s
        self.inches_to_meters = 0.0254
        wheelbase_width = 21.375 * self.inches_to_meters
        self.wheel_circumference = 13*math.pi*self.inches_to_meters
        self.counts = 512
        elaspedTime = 0.0
        motion_type = None
        current_x_m = 0.0
        current_y_m = 0.0
        current_theta = 0.0

        # Encoder Variables
        deltaM1_counts = 0
        deltaM2_count = 0
        counts_per_meter = (self.counts / self.wheel_circumference)
        meters_per_count = (self.wheel_circumference / self.counts)
        self.get_logger().info(f'{self.controller.ReadEncM1(self.address)}')
        m1_encoder_read = self.controller.ReadEncM1(self.address)
        m2_encoder_read = self.controller.ReadEncM2(self.address)
        if len(m1_encoder_read) !=3 or len(m2_encoder_read) != 3:
            goal_handle.abort()
            self.stop_motors()
            result.success = False
            return result
        
        else:
            m1_encoder_status, M1_encoder_start_count,_  = m1_encoder_read
            m2_encoder_status, M2_encoder_start_count,_  = m2_encoder_read

        M1_encoder_current_count = M1_encoder_start_count
        M2_encoder_current_count = M2_encoder_start_count
        M1_encoder_prev_count = M1_encoder_start_count
        M2_encoder_prev_count = M2_encoder_start_count
        tolerance = 10 # delta count tolerance

        # Stall Variables 
        stall_threshold = 0.5 #seconds
        stall_time = 0 #seconds
        stall_delta = 3 #counts

        # Result Functions
        def failed_result(motion_type):
            result.success = False

            if motion_type == "translation":
                result.final_pos.x = float(current_x_m)
                result.final_pos.y = 0.0
                result.final_pos.theta = 0.0
                result.final_time = elaspedTime
                return result
            
            elif motion_type == "rotation":
                result.final_pos.x = 0.0
                result.final_pos.y = 0.0
                result.final_pos.theta = math.degrees(current_theta)
                result.final_time = elaspedTime
                return result
        
        def successful_result(motion_type):
            result.success = True

            if motion_type == "translation":
                result.final_pos.x = float(current_x_m)
                result.final_pos.y = 0.0
                result.final_pos.theta = 0.0
                result.final_time = endTime-startTime
                return result
        
            elif motion_type == "rotation":
                result.final_pos.x = 0.0
                result.final_pos.y = 0.0
                result.final_pos.theta = math.degrees(current_theta)
                result.final_time = endTime-startTime
                return result
        
        # Pure Translation
        if (x != 0.0 and y == 0.0 and theta == 0.0 and ang_vel == 0.0):

            motion_type = "translation"
            delta_counts = int(round(x *counts_per_meter))
            M1_encoder_target_count =round( M1_encoder_start_count + delta_counts) 
            M2_encoder_target_count =round( M2_encoder_start_count + delta_counts)
            M1_speed_counts = round(vel * counts_per_meter)
            M2_speed_counts = M1_speed_counts
            self.expected_time = abs(x/vel) * 4

        # Pure Rotation
        elif (x == 0.0 and y == 0.0 and vel == 0.0 and theta != 0.0 and ang_vel != 0.0):

            motion_type = "rotation"
            radius = (wheelbase_width/2)
            rightwheel_velocity = ang_vel * radius
            leftwheel_velocity = -ang_vel * radius
            arclength = theta*radius
            self.expected_time = abs(theta/ang_vel) * 4
            
            delta_counts = int(round(arclength * counts_per_meter))
            M1_encoder_target_count = M1_encoder_start_count + delta_counts
            M2_encoder_target_count = M2_encoder_start_count - delta_counts
            M1_speed_counts = round(rightwheel_velocity * counts_per_meter)
            M2_speed_counts = round(leftwheel_velocity * counts_per_meter)
        
        else:
            goal_handle.abort()
            self.stop_motors()
            result.success = False
            return result

        # Begin Motion Control
        try:
            # Log Current and Target Encoder Count
            self.get_logger().info(f'Current M1 Encoder Count:{M1_encoder_start_count}')
            self.get_logger().info(f'Target M1 Encoder Count:{M1_encoder_target_count}')
            self.get_logger().info(f'Current M2 Encoder Count:{M2_encoder_start_count}')
            self.get_logger().info(f'Target M2 Encoder Count:{M2_encoder_target_count}')
            
            startTime = time.perf_counter()
            prevTime = startTime
            
            
            self.controller.SpeedM1M2(self.address,M1_speed_counts, M2_speed_counts)

            while (abs(M1_encoder_target_count - M1_encoder_current_count) > tolerance or abs(M2_encoder_target_count - M2_encoder_current_count) > tolerance):                    # Update Position and Velocity
                currentTime = time.perf_counter()
                elaspedTime = currentTime - startTime

                 # Cancel Request Check
                if (goal_handle.is_cancel_requested):
                    self.stop_motors()
                    goal_handle.canceled()
                    return failed_result(motion_type)

                # Velocity Check 
                m1_vel_status,current_m1_velocity,_ = self.controller.ReadSpeedM1(self.address) 
                m2_vel_status,current_m2_velocity,_ = self.controller.ReadSpeedM2(self.address)

                if not (m1_vel_status or m2_vel_status):
                    self.stop_motors()
                    goal_handle.abort()
                    return failed_result(motion_type)

                # Encoder Check
                M1status, M1_encoder_current_count, _ = self.controller.ReadEncM1(self.address)
                M2status, M2_encoder_current_count, _ = self.controller.ReadEncM2(self.address)
                if not (M1status and M2status):
                    self.stop_motors()
                    goal_handle.abort()
                    return failed_result(motion_type)
                    
                # Timeout Check
                if elaspedTime > self.expected_time:
                    self.stop_motors()
                    goal_handle.abort()
                    return failed_result(motion_type)
                    
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

                if motion_type == "translation":
                    
                    current_x_m = abs(M1_encoder_current_count- M1_encoder_start_count) * (self.wheel_circumference/self.counts)

                    # Feedback Update
                    feedback.current_pos.x = float(current_x_m)
                    feedback.current_pos.y = 0.0
                    feedback.current_pos.theta = 0.0
                    feedback.current_m1_velocity = float(current_m1_velocity)
                    feedback.current_m2_velocity = float(current_m2_velocity)
                    feedback.m1_encoder_count = int(M1_encoder_current_count)
                    feedback.m2_encoder_count = int(M2_encoder_current_count)
                    goal_handle.publish_feedback(feedback)
                    time.sleep(self.delay_time)

                elif motion_type == "rotation":
                    M1_current_distance = (M1_encoder_current_count - M1_encoder_start_count) / counts_per_meter
                    M2_current_distance = (M2_encoder_current_count - M2_encoder_start_count) / counts_per_meter
                    current_theta = (M1_current_distance - M2_current_distance) / wheelbase_width

                    # Feedback Update 
                    feedback.current_pos.x = 0.0
                    feedback.current_pos.y = 0.0
                    feedback.current_pos.theta = math.degrees(current_theta)
                    feedback.current_m1_velocity = float(current_m1_velocity)
                    feedback.current_m2_velocity = float(current_m2_velocity)
                    feedback.m1_encoder_count = int(M1_encoder_current_count)
                    feedback.m2_encoder_count = int(M2_encoder_current_count)
                    goal_handle.publish_feedback(feedback)
                    time.sleep(self.delay_time)

        except Exception as e:
            self.stop_motors()
            goal_handle.abort()
            self.get_logger().error(f"An exception occured: {e}")
            return failed_result(motion_type)
        
        finally:
            self.stop_motors()
        
        goal_handle.succeed()
        endTime = time.perf_counter()
        self.stop_motors()
        return successful_result(motion_type)
        
def main(args=None):
    rclpy.init(args=args)
    node = MoveToServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
