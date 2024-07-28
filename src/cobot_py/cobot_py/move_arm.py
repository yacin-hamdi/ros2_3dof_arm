import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math



class MoveArm(Node):
    def __init__(self):
        super().__init__("move_arm")
        self.move_publisher = self.create_publisher(JointTrajectory, "set_joint_trajectory", 10)
        
        self.get_logger().info("move_arm publisher has been started")
        self.solid_2_joint_angle_ = .0
        self.solid_1_joint_angle_ = .0
        self.base_joint_1_ = .0
        self.x_ = 400.0
        self.y_ = 400.0

        self.a = 800.0

        self.state = True

        self.timer_ = self.create_timer(1., self.move_rectangle)



    def publish_pos(self):
        # if self.x_ >= 1350.0:
        #     self.state = False
        # elif self.x_ <= 10.0:
        #     self.state = True

        # if self.state:
        #     self.x_ += 1
        # else:
        #     self.x_ -= 1
        q1, q2 = self.inverse_kinematic(self.x_, self.y_)
        msg = JointTrajectory()
        msg.header.frame_id = "world"
        msg.joint_names = ["base_joint_1", "solid_1_joint", "solid_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [self.base_joint_1_, q1, q2]
        msg.points = [point]
        self.move_publisher.publish(msg=msg)

    def move_rectangle(self):
        while self.x_ >= 400.0:
            self.x_ -= 1
            self.publish_pos()
        
        while self.y_ <=1000.0:
            self.y_ += 1
            self.publish_pos()

        while self.x_ <= 1000.0:
            self.x_ += 1
            self.publish_pos()
        
        while self.y_ >= 400.0:
            self.y_ -= 1
            self.publish_pos()

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

