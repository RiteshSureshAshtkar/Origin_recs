#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from pynav_interfaces.action import PynavGoal
from rclpy.action import ActionClient
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from tf_transformations import euler_from_quaternion

class RvizBridge(Node):
    def __init__(self):
        super().__init__("rviz_bridge")
        
        # Subscribe to RViz Initial Pose
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            10
        )
        
        # Subscribe to RViz Goal Pose
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            10
        )
        
        # Action Client for Navigation
        self.action_client = ActionClient(self, PynavGoal, "pynav/navigation_goal")
        
        # Service Client for setting initial_pose parameter on a_star_planner
        self.param_client = self.create_client(SetParameters, "/a_star_planner/set_parameters")
        
        self.get_logger().info("RVIZ BRIDGE STARTED")

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info(f"Received Initial Pose: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("a_star_planner/set_parameters service not available")
            return

        req = SetParameters.Request()
        
        # Construct the parameter
        param_value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[msg.pose.pose.position.x, msg.pose.pose.position.y])
        param = Parameter(name="initial_pose", value=param_value)
        req.parameters = [param]
        
        future = self.param_client.call_async(req)
        future.add_done_callback(self.param_callback)

    def param_callback(self, future):
        try:
            result = future.result()
            if result.results[0].successful:
                self.get_logger().info("Successfully set initial_pose")
            else:
                self.get_logger().warn(f"Failed to set initial_pose: {result.results[0].reason}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received Goal Pose: {msg.pose.position.x}, {msg.pose.position.y}")
        
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        theta = euler_from_quaternion([x, y, z, w])[2]
        
        goal_msg = PynavGoal.Goal()
        goal_msg.goal_pose_x = [float(msg.pose.position.x)]
        goal_msg.goal_pose_y = [float(msg.pose.position.y)]
        goal_msg.goal_pose_theta = [float(theta)]  # Using a single point for now
        
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RvizBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
