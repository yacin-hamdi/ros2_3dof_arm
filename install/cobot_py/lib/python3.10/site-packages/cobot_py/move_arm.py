import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint



class MoveArm(Node):
    def __init__(self):
        super().__init__("move_arm")
        self.move_publisher = self.create_publisher(JointTrajectory, "set_joint_trajectory", 10)
        self.timer_ = self.create_timer(1.0, self.publish_pos)
        self.get_logger().info("move_arm publisher has been started")
        self.solid_2_joint_pos_ = .0

    def publish_pos(self):
        msg = JointTrajectory()
        msg.header.frame_id = "world"
        msg.joint_names = ["base_joint_1", "solid_1_joint", "solid_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0., 0., self.solid_2_joint_pos_]
        msg.points = [point]
        self.move_publisher.publish(msg=msg)
        self.solid_2_joint_pos_ += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = MoveArm()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

