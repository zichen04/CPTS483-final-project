import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

JOINT_NAMES = [
    'shoulder_abduct',
    'shoulder_flex',
    'elbow_flex',
    'forearm_pronate',
]

class JointPublisher(Node):

    def __init__(self, angle_queue):
        super().__init__('arm_joint_publisher')
        self.pub    = self.create_publisher(JointState, '/joint_states', 10)
        self.queue  = angle_queue
        self.angles = {name: 0.0 for name in JOINT_NAMES}
        self.timer  = self.create_timer(1.0 / 30.0, self.publish_joints)
        self.get_logger().info('joint publisher ready')

    def publish_joints(self):
        while not self.queue.empty():
            new_angles = self.queue.get_nowait()
            self.angles.update(new_angles)

        msg              = JointState()
        msg.header       = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = JOINT_NAMES
        msg.position     = [self.angles[n] for n in JOINT_NAMES]
        msg.velocity     = []
        msg.effort       = []
        self.pub.publish(msg)