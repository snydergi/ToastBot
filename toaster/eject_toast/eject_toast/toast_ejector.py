import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ToastEjector(Node):
    def __init__(self):
        super().__init__('toast_ejector')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)  # 0.1s timer (10Hz)
        
        self.ejection_height = 0.0
        self.direction = 1  # Moving up initially
        self.state = 'moving_up'  # Initial state
        self.count = 0  # Used for timing delays

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['toast_joint']
        joint_state.position = [self.ejection_height]

        if self.state == 'moving_up':
            self.ejection_height += 0.005  # Increment height
            if self.ejection_height >= 0.11:  # Upper limit
                self.state = 'waiting_at_top'
                self.count = 0  # Reset count for the wait time

        elif self.state == 'waiting_at_top':
            self.count += 1
            if self.count >= 50:  # Wait for 5 cycles (0.1s * 5 = 0.5 seconds)
                self.state = 'moving_down'

        elif self.state == 'moving_down':
            self.ejection_height -= 0.005  # Decrement height
            if self.ejection_height <= -0.1:  # Lower limit
                self.state = 'waiting_at_bottom'
                self.count = 0  # Reset count for the wait time

        elif self.state == 'waiting_at_bottom':
            self.count += 1
            if self.count >= 20:  # Wait for 2 seconds (0.1s * 20 = 2 seconds)
                self.state = 'moving_up'

        # Publish the joint state
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = ToastEjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
