# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from autoware_control_msgs.msg import AckermannControlCommand
# from geometry_msgs.msg import Twist
# import math

# class AckermannControlCommand_to_cmd_vel(Node):
#     def __init__(self,node_name):
#         super().__init__(node_name)
#         self.get_logger().info("Ackermann Control Command to cmd_vel .")
#         self.AckermannControlCommand_msg = AckermannControlCommand()
#         self.Cmd_vel_msg = Twist()

#         self.AckermannControlCommand_sub = self.create_subscription(AckermannControlCommand,"/control/command/control_cmd",self.AckermannControlCommand_sub_callback,10)
#         self.cmd_vel_pub = self.create_publisher(Twist,"cmd_vel",10)

#     def AckermannControlCommand_sub_callback(self,date):
#         # 根据阿克曼公式计算角速度
#         wheelbase = 1.52  # 小车轮距
#         wheelD = 0.62     # 轮直径
#         lenbase = 2.86    # 小车轴距
#         # 角速度 = （ tan(内轮转角) * 车子线速度 ）/ 车子轴距
#         self.AckermannControlCommand_msg.longitudinal.speed = date.longitudinal.speed
#         self.AckermannControlCommand_msg.lateral.steering_tire_angle = date.lateral.steering_tire_angle
#         # print(self.AckermannControlCommand_msg.longitudinal.speed," - ",self.AckermannControlCommand_msg.longitudinal.acceleration)
#         self.Cmd_vel_msg.linear.x = self.AckermannControlCommand_msg.longitudinal.speed
#         # self.Cmd_vel_msg.angular.z = self.AckermannControlCommand_msg.lateral.steering_tire_angle
#         if self.AckermannControlCommand_msg.longitudinal.speed != 0 and self.AckermannControlCommand_msg.lateral.steering_tire_angle != 0 :
#             self.Cmd_vel_msg.angular.z = math.tan(self.AckermannControlCommand_msg.lateral.steering_tire_angle) * self.AckermannControlCommand_msg.longitudinal.speed / lenbase
#         else :
#             self.Cmd_vel_msg.angular.z = 0.0
#         self.cmd_vel_pub.publish(self.Cmd_vel_msg)
    
#     def degree2angular(self) :
#         pass


# def main(args=None):
#     rclpy.init(args=args)			    
#     AckermannControlCommand_to_cmd_vel_node = AckermannControlCommand_to_cmd_vel("AckermannControlCommand_to_cmd_vel_node")    
#     rclpy.spin(AckermannControlCommand_to_cmd_vel_node)                 
#     rclpy.shutdown()

# main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control  # 修改：使用正确的消息类型
from geometry_msgs.msg import Twist
import math

class ControlCommand_to_cmd_vel(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Control Command to cmd_vel converter started.")
        
        # 初始化消息
        self.control_msg = Control()
        self.cmd_vel_msg = Twist()

        # 车辆参数
        self.wheelbase = 0.462  # 小车轮距 (m)
        self.wheelD = 0.264     # 轮直径 (m) 
        self.lenbase = 0.501    # 小车轴距 (m)
        
        # 订阅控制命令话题
        self.control_sub = self.create_subscription(
            Control,
            "/control/command/control_cmd",
            self.control_callback,
            10
        )
        
        # 发布cmd_vel话题
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        self.get_logger().info(f"Vehicle parameters - Wheelbase: {self.wheelbase}m, Axle length: {self.lenbase}m")

    def control_callback(self, msg):
        """
        处理Control消息并转换为cmd_vel
        Control消息结构:
        - longitudinal.velocity: 纵向速度 (m/s)
        - longitudinal.acceleration: 纵向加速度 (m/s²)
        - lateral.steering_tire_angle: 转向轮角度 (rad)
        """
        try:
            # 获取纵向速度和转向角
            linear_velocity = msg.longitudinal.velocity
            steering_angle = msg.lateral.steering_tire_angle
            if abs(msg.longitudinal.velocity) > 0.01:
                self.get_logger().info(
                    f"autoware-------Control -> velocity={msg.longitudinal.velocity:.3f}m/s,"
                    f"steering_tire_angle={msg.lateral.steering_tire_angle:.10f}°,"
                )

            # 设置线速度
            self.cmd_vel_msg.linear.x = linear_velocity
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.linear.z = 0.0
            
            # 计算角速度：ω = v * tan(δ) / L
            # 其中：v = 线速度，δ = 转向角，L = 轴距
            if abs(linear_velocity) > 0.01 and abs(steering_angle) > 0.001:
                # 防止除零和过小值
                angular_velocity = (linear_velocity * math.tan(steering_angle)) / self.lenbase
                
                # 限制角速度范围 (防止过大的角速度)
                max_angular_vel = 2.0  # rad/s
                angular_velocity = max(-max_angular_vel, min(max_angular_vel, angular_velocity))
                
                self.cmd_vel_msg.angular.z = angular_velocity
            else:
                self.cmd_vel_msg.angular.z = 0.0
            
            # 清零其他角速度分量
            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            
            # 发布cmd_vel消息
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            
            # 调试信息 (可选)
            if abs(linear_velocity) > 0.01:
                self.get_logger().info(
                    f"Control -> cmd_vel: v={linear_velocity:.3f}m/s, "
                    f"steering={math.degrees(steering_angle):.2f}°, "
                    f"ω={self.cmd_vel_msg.angular.z:.10f}rad/s"
                )
                
        except Exception as e:
            self.get_logger().error(f"Error in control_callback: {e}")
            # 发送停止命令
            self.cmd_vel_msg = Twist()  # 全零消息
            self.cmd_vel_pub.publish(self.cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        control_to_cmd_vel_node = ControlCommand_to_cmd_vel("control_to_cmd_vel_node")
        rclpy.spin(control_to_cmd_vel_node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

