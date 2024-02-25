import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import sys
import cv2
import os

from load_img.lib import get_path

#---TODO-------------------------------------
IMAGE_DIRECTORY_PATH = get_path("Collected_Datasets")
SHOW_IMAGE = True
TIMER = 0.1
QUE = 1
#--------------------------------------------

class ImagePublisherNode(Node):
    def __init__(self, img_path=IMAGE_DIRECTORY_PATH, pub_topic='topic_raw_img', logger=SHOW_IMAGE, timer=TIMER, que=QUE):
        super().__init__('node_pub_img')
        
        self.declare_parameter('img_path', img_path)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('logger', logger)
        self.declare_parameter('timer', timer)
        self.declare_parameter('que', que)
        
        self.img_path = self.get_parameter('img_path').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer').get_parameter_value().double_value
        self.que = self.get_parameter('que').get_parameter_value().integer_value
        
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(self.img_path)
        
        self.publisher = self.create_publisher(Image, self.pub_topic, self.que)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.path = get_path(self.img_path)
        self.img_list = sorted(os.listdir(self.path), key=lambda x: int(x.split('_')[0]))
        self.img_num = 0
        
    def timer_callback(self):         
        try:
            if self.img_num < len(self.img_list):
                img_file = self.img_list[self.img_num]
                img_path = os.path.join(self.path, img_file) 
                img = cv2.imread(img_path)
                self.publisher.publish(self.br.cv2_to_imgmsg(img))
                self.img_num += 1  
            
                if self.logger:
                    self.get_logger().info('Saved image: %s' % img_file)
                    cv2.imshow('Saved image', img)
                    cv2.waitKey(1)
                
            else:
                print('\n\nNo more images\n\n')
                self.img_num = 0
                
        except CvBridgeError as e:
            self.timer.cancel()
            self.cap.release()
            cv2.destroyAllWindows()
            sys.exit(0)  
      
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
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
