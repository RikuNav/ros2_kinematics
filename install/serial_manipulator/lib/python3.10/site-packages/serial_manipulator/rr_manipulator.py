import rclpy
from rclpy.node import Node
from .submodules.robo_class import SimpleRobot
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from .submodules.functions import quaternion_from_matrix

class RRManipulator(Node):
    def __init__(self):
        super().__init__('rr_manipulator')

        offsets = {'1': ["R", [0, 1, 1, 90]],
                   '2': ["R", [0, 1, 1, 0]]}

        self.change = True
        self.robot = SimpleRobot(**offsets)

        self.subscription = self.create_subscription(Float64MultiArray, 'joint_data', self.joint_callback, 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint = TransformStamped()

        self.get_logger().info('RR Manipulator Node has been started')

    def joint_callback(self, msg):
        self.change = True
        self.robot(msg.data)

    def timer_callback(self):
        for idx, output in enumerate(self.robot.outputs):
            self.joint.header.stamp = self.get_clock().now().to_msg()
            self.joint.transform.translation.x = output[0, 3]
            self.joint.transform.translation.y = output[1, 3]
            self.joint.transform.translation.z = output[2, 3]

            q = quaternion_from_matrix(self.robot.outputs[idx])
            self.joint.transform.rotation.x = q[0]
            self.joint.transform.rotation.y = q[1]
            self.joint.transform.rotation.z = q[2]
            self.joint.transform.rotation.w = q[3]

            if idx < len(self.robot.outputs) - 1:
                self.joint.header.frame_id = f'joint{idx}'
                self.joint.child_frame_id = f'joint{idx + 1}'
            else:
                self.joint.header.frame_id = f'joint{idx}'
                self.joint.child_frame_id = 'end_effector'

            self.tf_broadcaster.sendTransform(self.joint)


def main(args=None):
    rclpy.init(args=args)
    tf_rr = RRManipulator()
    rclpy.spin(tf_rr)
    tf_rr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()