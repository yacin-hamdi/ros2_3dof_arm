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
        
        self.x_ = 400.0
        self.y_ = 0.0
        self.z_ = 1000.0

        self.l1 = 800.0
        self.l2 = 800.0
        self.l3 = 800.0


        self.timer_ = self.create_timer(0.01, self.move_rectangle)



    def publish_pos(self):
       

        q1, q2, q3 = self.inverse_kinematic(self.x_, self.y_, self.z_)
        msg = JointTrajectory()
        msg.header.frame_id = "world"
        msg.joint_names = ["base_joint_1", "solid_1_joint", "solid_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [q1, q2, q3]
        msg.points = [point]
        self.move_publisher.publish(msg=msg)

    def move_rectangle(self):
        while self.x_ >= 400.0:
            self.x_ -= 1
            self.publish_pos()
        
        while self.y_ <=800.0:
            self.y_ += 1
            self.publish_pos()

        while self.x_ <= 800.0:
            self.x_ += 1
            self.publish_pos()
        
        while self.y_ >= 400.0:
            self.y_ -= 1
            self.publish_pos()

    

    def inverse_kinematic(self, x, y, z):
        h = math.sqrt(x*x + y*y)
        theta1 = math.atan2(y, x)
        
        r_2 = math.pow(z-self.l1, 2) + h*h
        alpha = math.acos((self.l2*self.l2 + self.l3*self.l3 - r_2)/(2*self.l2*self.l3))
        theta3 = math.pi - alpha
        beta = math.atan2(z-self.l1, h)
        phi = math.atan2(self.l3 * math.sin(theta3), self.l2 + self.l3*math.cos(theta3))
        theta2 = beta - phi

        self.get_logger().info(f"q1:{theta1}, q2:{theta2}, q3:{theta3}")
        return theta1, theta2, theta3



def main(args=None):
    rclpy.init(args=args)
    node = MoveArm()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

