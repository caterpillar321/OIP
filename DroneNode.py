import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from std_srvs.srv import Empty 
import math
from rclpy.qos import qos_profile_sensor_data

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')

        # roll, pitch, yaw 기본값
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.cmd_count = 0
        self.pose_count = 0

        self.velocity_publisher = self.create_publisher(
            TwistStamped, 
            '/mavros/setpoint_velocity/cmd_vel', 
            10
        )
        self.get_logger().info("Velocity publisher initialized.")

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.cmd_subscriber = self.create_subscription(
            Twist,
            'drone_cmd',
            self.cmd_callback,
            10
        )

        self.land_subscriber = self.create_subscription(
            String,
            'drone_land',
            self.land_order_callback,
            10
        )

        # 드론 상태(roll, pitch, yaw) 퍼블리시
        self.drone_info_publisher = self.create_publisher(Vector3, 'drone_info', 10)

        # 서비스 클라이언트 준비(OFFBOARD 모드, Arming, 착륙)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        # 서비스 서버 준비 기다리기
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/land service...')
       

        self.create_timer(0.1, self.publish_drone_info)

    def pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation

        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.roll = math.degrees(roll)
        self.pitch = math.degrees(pitch)
        self.yaw = math.degrees(yaw)

        self.pose_count += 1
        if self.pose_count % 50 == 0:
            self.get_logger().info(f"Current RPY(deg): roll={self.roll:.1f}, pitch={self.pitch:.1f}, yaw={self.yaw:.1f}")

    def cmd_callback(self, msg: Twist):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.linear.z
        yaw_rate = msg.angular.z

        self.cmd_count += 1
        self.move_drone(x, y, z, yaw_rate)

        if self.cmd_count == 5:
            self.enable_offboard_mode()
            self.arm_drone()

    def move_drone(self, x, y, z, yaw_rate):
        yaw_rad = math.radians(self.yaw)

        x_global = x * math.cos(yaw_rad) - y * math.sin(yaw_rad)
        y_global = x * math.sin(yaw_rad) + y * math.cos(yaw_rad)

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = self.get_clock().now().to_msg()
        velocity_msg.twist.linear.x = x_global
        velocity_msg.twist.linear.y = y_global
        velocity_msg.twist.linear.z = z
        velocity_msg.twist.angular.z = yaw_rate

        self.velocity_publisher.publish(velocity_msg)

    def enable_offboard_mode(self):
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self.offboard_callback)

    def offboard_callback(self, future):
        try:
            result = future.result()
            if result is not None and result.mode_sent:
                self.get_logger().info("OFFBOARD 모드로 변경됨!")
            else:
                self.get_logger().error("OFFBOARD 모드 변경 실패!")
        except Exception as e:
            self.get_logger().error(f"OFFBOARD 모드 요청 중 예외 발생: {str(e)}")

    def arm_drone(self):
        request = CommandBool.Request()
        request.value = True
        future = self.arming_client.call_async(request)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info("드론이 Arming 됨!")
            else:
                self.get_logger().error("드론 Arming 실패!")
        except Exception as e:
            self.get_logger().error(f"드론 Arming 요청 중 예외 발생: {str(e)}")

    def land_drone(self):
        self.get_logger().info("Landing Drone...")
        request = CommandTOL.Request()
        future = self.land_client.call_async(request)
        future.add_done_callback(self.land_callback)

    def land_callback(self, future):
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info("드론이 착륙 중...")
            else:
                self.get_logger().error("드론 착륙 실패!")
        except Exception as e:
            self.get_logger().error(f"드론 착륙 요청 중 예외 발생: {str(e)}")

    def land_order_callback(self, msg : String):
        if msg.data == "land":
            self.get_logger().info("Landing command received, executing land_drone()")
            self.land_drone()

    def publish_drone_info(self):
        msg = Vector3()
        msg.x = self.roll
        msg.y = self.pitch
        msg.z = self.yaw
        self.drone_info_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
