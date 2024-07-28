import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math



class MoveArm(Node):
    def __init__(self):
        super().__init__("move_arm")
        self.move_publisher = self.create_publisher(JointTrajectory, "set_joint_trajectory", 10)
        self.timer_ = self.create_timer(.005, self.publish_pos)
        self.get_logger().info("move_arm publisher has been started")
        self.solid_2_joint_angle_ = .0
        self.solid_1_joint_angle_ = .0
        self.base_joint_1_ = .0
        self.var = 800.0

        self.a = 800.0

        self.state = True


    def publish_pos(self):
        if self.var >= 1350.0:
            self.state = False
        elif self.var <= 10.0:
            self.state = True

        if self.state:
            self.var += 1
        else:
            self.var -= 1
        q1, q2 = self.inverse_kinematic(self.var, 800.)
        msg = JointTrajectory()
        msg.header.frame_id = "world"
        msg.joint_names = ["base_joint_1", "solid_1_joint", "solid_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [self.base_joint_1_, q1, q2]
        msg.points = [point]
        self.move_publisher.publish(msg=msg)

    def inverse_kinematic(self, x, y):
        q2 = math.acos((x*x + y*y - self.a*self.a - self.a*self.a)/(2*self.a*self.a))

        q1 = math.atan(y/x) - math.atan(self.a * math.sin(q2)/(self.a + self.a*math.cos(q2)))

        
        self.get_logger().info(f"q1:{q1}, q2:{q2}")
        return q1, q2



def main(args=None):
    rclpy.init(args=args)
    node = MoveArm()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

