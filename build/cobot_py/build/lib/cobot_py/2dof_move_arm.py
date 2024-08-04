import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math



class MoveArm2DOF(Node):
    def __init__(self):
        super().__init__("move_arm_2dof")
        self.move_publisher = self.create_publisher(JointTrajectory, "set_joint_trajectory", 10)
        
        self.get_logger().info("move_arm publisher has been started")
        self.solid_2_joint_angle_ = .0
        self.solid_1_joint_angle_ = .0
        self.base_joint_1_ = .0
        self.x_ = 1100.0
        self.y_ = 1100.0

        self.l1 = 800.0
        self.l2 = 800.0
        self.l3 = 800.0

        self.state = True

        self.timer_ = self.create_timer(1, self.move_rectangle)



    def publish_pos(self):
        q1, q2= self.inverse_kinematic(self.x_, self.y_)
        msg = JointTrajectory()
        msg.header.frame_id = "world"
        msg.joint_names = ["base_joint_1", "solid_1_joint", "solid_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, q1, q2]
        msg.points = [point]
        self.move_publisher.publish(msg=msg)

    def move_rectangle(self):
        while self.x_ >= 400.0:
            self.x_ -= 1
            self.publish_pos()
        
        while self.y_ <=1800.0:
            self.y_ += 1
            self.publish_pos()

        while self.x_ <= 1000.0:
            self.x_ += 1
            self.publish_pos()
        
        while self.y_ >= 1200.0:
            self.y_ -= 1
            self.publish_pos()

    def inverse_kinematic(self, x, y):
        

        r_2 = math.pow(y-self.l1, 2) + x*x
        alpha = math.acos((self.l2*self.l2 + self.l3*self.l3 - r_2)/(2*self.l2*self.l3))
        theta2 = math.pi - alpha

        psi = math.atan2(self.l3*math.sin(theta2), self.l2 + self.l3*math.cos(theta2))
        beta = math.atan2(y-self.l1, x)
        theta1 = beta - psi


        
        self.get_logger().info(f"q1:{theta1}, q2:{theta2}")
        return theta1, theta2



def main(args=None):
    rclpy.init(args=args)
    node = MoveArm2DOF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

