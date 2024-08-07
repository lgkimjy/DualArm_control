
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(Float64MultiArray, '/position_controllers/commands', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(100)

        index = 0

        # robot state
        delta = 0.01
        clavicle_pos = 0.0
        shoulder_left_pos = 0.0

        # message declarations
        joint_state = Float64MultiArray()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                joint_state.data = [clavicle_pos, shoulder_left_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
                # send the joint state
                self.joint_pub.publish(joint_state)

                # Create new robot state
                if index == 0:
                    clavicle_pos = clavicle_pos + delta
                    shoulder_left_pos = shoulder_left_pos + delta
                    if clavicle_pos >= 1:
                        index = 1
                elif index == 1:
                    clavicle_pos = clavicle_pos - delta
                    shoulder_left_pos = shoulder_left_pos - delta
                    if clavicle_pos <= -1:
                        index = 0
                
                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()