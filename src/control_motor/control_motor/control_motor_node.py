import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from interfaces_pkg.msg import Motor

from .lib import control_motor as CONTROL
from .lib import convert_arduino_msg as PROTOCOL

#---TODO-------------------------------------
SUB_TOPIC_NAME = "topic_protocol_signal"

TIMER = 0.1
QUE = 1

PORT='/dev/ttyACM0'

VARIBLE_RESISTER_INPUT_PIN = 'A2'

STEERING_PIN1 = 10
STEERING_PIN2 = 12
LEFT_REAR_PIN1 = 6
LEFT_REAR_PIN2 = 8
RIGHT_REAR_PIN1 = 3
RIGHT_REAR_PIN2 = 5

resistance_most_left = 433
resistance_most_right = 309

#--------------------------------------------

ser = serial.Serial(PORT, 9600, timeout=1)
time.sleep(1)

class SendSignal():
  def __init__(self):
    CONTROL.arduino_pinsetting(ser,
                        VARIBLE_RESISTER_INPUT_PIN,
                        STEERING_PIN1,
                        STEERING_PIN2,
                        LEFT_REAR_PIN1,
                        LEFT_REAR_PIN2,
                        RIGHT_REAR_PIN1,
                        RIGHT_REAR_PIN2,
                        resistance_most_left,
                        resistance_most_right)    
    time.sleep(1)

  def process(self, protocol):
    message = protocol.data
    ser.write(message.encode())
    return

class MotorControlNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME, pub_topic="topic_send_signal", timer=TIMER, que=QUE):
    super().__init__('node_control_motor')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('pub_topic', pub_topic)
    self.declare_parameter('timer', timer)
    self.declare_parameter('que', que)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
    self.timer_period = self.get_parameter('timer').get_parameter_value().double_value
    self.que = self.get_parameter('que').get_parameter_value().integer_value

    self.send_serial = SendSignal()
    
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=self.que)
    self.subscription = self.create_subscription(String, self.sub_topic, self.data_callback, qos_profile)
    self.publisher_ = self.create_publisher(Motor, self.pub_topic , self.que)

  def data_callback(self, msg):
    self.send_serial.process(msg)

def main(args=None):
  rclpy.init(args=args)
  node = MotorControlNode()
  try:
      rclpy.spin(node)
      
  except KeyboardInterrupt:
      print("\n\nshutdown\n\n")
      message = PROTOCOL.protocol_with_differential(0, 0, 0)
      ser.write(message.encode())
      pass
    
  finally:
    ser.close()
    print('closed')
    
  node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
