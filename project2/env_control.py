import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PreciseTrafficController(Node):
    def __init__(self):
        super().__init__('precise_traffic_controller')
        self.publisher = self.create_publisher(Float64, '/traffic_light/cmd_pos', 10)

        # 7초마다 타이머 실행
        self.timer_period = 7.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.is_red_showing = True
        self.get_logger().info("Traffic Light Controller Initialized (7s Cycle)")

    def timer_callback(self):
        msg = Float64()

        if self.is_red_showing:
            # 가림막을 아래로 내려서 초록불을 가리고 빨간불을 보여줌
            msg.data = -0.015
            self.get_logger().info("Changing to: RED")
        else:
            # 가림막을 위로 올려서 빨간불을 가리고 초록불을 보여줌
            msg.data = 0.015
            self.get_logger().info("Changing to: GREEN")

        self.publisher.publish(msg)
        self.is_red_showing = not self.is_red_showing

def main(args=None):
    rclpy.init(args=args)
    node = PreciseTrafficController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
