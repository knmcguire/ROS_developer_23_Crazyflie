import rclpy

class CrazyflieDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        rclpy.init(args=None)
        self.node = rclpy.create_node('crazyflie_webots_driver')

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
