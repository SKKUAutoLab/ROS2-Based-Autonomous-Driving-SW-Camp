import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from interfaces_pkg.msg import Motor

from .lib import convert_arduino_msg as PROTOCOL

#---TODO-------------------------------------
SUB_TOPIC_NAME = "topic_contol_signal"
PUB_TOPIC_NAME = "topic_protocol_signal"

TIMER = 0.1
QUE = 1
#--------------------------------------------

class ConvertProtocol():
  def __init__(self):
    pass
  
  def process(self, motor):
    steer = motor.angle
    left_speed = motor.left_speed
    right_speed = motor.right_speed 
    
    message = PROTOCOL.protocol_with_differential(steer, left_speed, right_speed)
    
    print(message)
    return message
  
class ConvertProtocolNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME, pub_topic=PUB_TOPIC_NAME, timer=TIMER, que=QUE):
    super().__init__('node_convert_protocol')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('pub_topic', pub_topic)
    self.declare_parameter('timer', timer)
    self.declare_parameter('que', que)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
    self.timer_period = self.get_parameter('timer').get_parameter_value().double_value
    self.que = self.get_parameter('que').get_parameter_value().integer_value

    self.is_running = False
    self.convert = ConvertProtocol()
    
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=self.que)
    self.subscription = self.create_subscription(Motor, self.sub_topic, self.data_callback, qos_profile)
    self.publisher_ = self.create_publisher(String, self.pub_topic , self.que)
    
      
  def data_callback(self, motor):
    protocol_msg = self.convert.process(motor)
    
    msg = String()
    msg.data = protocol_msg
    self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConvertProtocolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        pass
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()