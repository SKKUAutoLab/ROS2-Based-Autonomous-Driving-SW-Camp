import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

#---TODO-------------------------------------
SUB_TOPIC_NAME = 'topic_raw_img'
QUE = 10
#--------------------------------------------

class ImageSubscriberNode(Node):
    def __init__(self, sub_topic=SUB_TOPIC_NAME, que=QUE):
        super().__init__('node_sub_img')
        
        self.declare_parameter('sub_topic', sub_topic)
        self.declare_parameter('que', que)
                
        self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        self.que = self.get_parameter('que').get_parameter_value().integer_value
        
        image_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=self.que)
        self.subscription = self.create_subscription(Image, self.sub_topic, self.image_callback, image_qos_profile)
        
        self.br = CvBridge()
        self.is_running = False
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def image_callback(self, data):
        self.is_running = True
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow(self.sub_topic, current_frame)
        cv2.waitKey(1)
    
    def timer_callback(self):
        if not self.is_running:
            self.get_logger().info('Not published yet: "%s"' % self.sub_topic)
        
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
