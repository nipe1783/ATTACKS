import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
from px4_msgs.msg import VehicleAttitude,BatteryStatus,VehicleGlobalPosition, VehicleLocalPosition  # Make sure this matches the actual message type


class VehicleAttitudeSubscriber(Node):
    def __init__(self):
        super().__init__('vehicle_attitude_subscriber')

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            VehicleAttitude, 
            '/fmu/out/vehicle_attitude', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'{msg.q}')
        self.quat = msg.q
        
class BatteryStatusSubscriber(Node):
    def __init__(self):
        super().__init__('battery_status_subscriber')

        self.battery_remaining = 0

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            BatteryStatus, 
            '/fmu/out/battery_status', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info(f'Voltage: {msg.voltage_v} V\n')
        # self.get_logger().info(f'Current: {msg.current_a} A\n')
        self.get_logger().info(f'{msg.remaining*100}')
        self.battery_remaining = msg.remaining*100
                
class VehicleGlobalPositionSubscriber(Node):
    def __init__(self):
        super().__init__('vehicle_global_position_subscriber')

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            VehicleGlobalPosition, 
            '/fmu/out/vehicle_global_position', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Lattitude: {msg.lat}\n')
        self.get_logger().info(f'Longitude: {msg.lon}\n')
        self.get_logger().info(f'Altitude: {msg.alt}\n')


class VehicleLocalPositionSubscriber(Node):
    def __init__(self):
        super().__init__('vehicle_local_position_subscriber')

        custom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC
        )

        self.subscription = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.listener_callback, 
            custom_qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'X: {msg.x}\n')
        self.get_logger().info(f'Y: {msg.y}\n')
        self.get_logger().info(f'Z: {msg.z}\n')
        self.x_pos = msg.x
        self.y_pos = msg.y
        self.z_pos = msg.z
        self.x_vel = msg.vx
        self.y_vel = msg.vy
        self.z_vel = msg.vz




def main(args=None):
    rclpy.init(args=args)
    
    attitude_node = VehicleAttitudeSubscriber()
    battery_node = BatteryStatusSubscriber()
    vehicle_global_pos_node = VehicleGlobalPositionSubscriber()
    vehicle_local_pos_node = VehicleLocalPositionSubscriber()
   
    while True: 
        try:
            rclpy.spin_once(attitude_node)
            rclpy.spin_once(battery_node)
            rclpy.spin_once(vehicle_global_pos_node)
            rclpy.spin_once(vehicle_local_pos_node)
        except KeyboardInterrupt:
            # Code to handle any other type of exception
            print("Exiting UI ROS Test...")
            attitude_node.destroy_node()
            battery_node.destroy_node()
            vehicle_global_pos_node.destroy_node()
            vehicle_local_pos_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()