import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniButtonEvent
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class PhantomJointTeleop(Node):
    def __init__(self):
        super().__init__('phantom_joint_teleop')

        self.bot = InterbotixManipulatorXS("rx200", "arm", "gripper")
        self.bot.arm.go_to_home_pose()

        self.prev_joint_positions = None
        self.gripper_closed = False
        self.prev_grey_state = 0
        self.prev_white_state = 0

        self.joint_sub = self.create_subscription(
            JointState,
            '/phantom/joint_states',
            self.joint_callback,
            10
        )

        self.button_sub = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.button_callback,
            10
        )

        self.get_logger().info("Phantom Joint Teleop Node Started.")

    def joint_callback(self, msg: JointState):
        if len(msg.position) < 6:
            self.get_logger().warn("Incomplete joint states received.")
            return

        phantom_joints = {
            name: pos for name, pos in zip(msg.name, msg.position)
        }

        # Mapping Phantom joints to RX200
        rx200_joint_positions = {
            'waist': -self.scale_joint(phantom_joints.get('waist', 0.0), -1.5, 1.5),
            'shoulder': -self.scale_joint(phantom_joints.get('shoulder', 0.0), -1.0, 1.0),
            'elbow': -self.scale_joint(phantom_joints.get('elbow', 0.0), -1.5, 1.5),
            'wrist_angle': -self.scale_joint(phantom_joints.get('yaw', 0.0), -1.0, 1.0),
            'wrist_rotate': -self.scale_joint(phantom_joints.get('pitch', 0.0), -1.0, 1.0)
        }

        current_joint_values = tuple(round(rx200_joint_positions[j], 2) for j in rx200_joint_positions)

        if current_joint_values == self.prev_joint_positions:
            return  # Skip if no change

        self.prev_joint_positions = current_joint_values

        try:
            self.bot.arm.set_joint_positions(list(rx200_joint_positions.values()))
            self.get_logger().info(f"Moved to joints: {rx200_joint_positions}")
        except Exception as e:
            self.get_logger().warn(f"Failed to move arm: {e}")

    def scale_joint(self, val, min_limit, max_limit):
        # Clamp to safety range
        return max(min_limit, min(max_limit, val))

    def button_callback(self, msg: OmniButtonEvent):
        grey = msg.grey_button
        white = msg.white_button

        if grey == 1 and self.prev_grey_state == 0:
            if not self.gripper_closed:
                self.bot.gripper.grasp(delay=0.0)
                self.get_logger().info("Gripper closed.")
                self.gripper_closed = True

        if white == 1 and self.prev_white_state == 0:
            if self.gripper_closed:
                self.bot.gripper.release(delay=0.0)
                self.get_logger().info("Gripper opened.")
                self.gripper_closed = False

        self.prev_grey_state = grey
        self.prev_white_state = white

def main(args=None):
    rclpy.init(args=args)
    node = PhantomJointTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

