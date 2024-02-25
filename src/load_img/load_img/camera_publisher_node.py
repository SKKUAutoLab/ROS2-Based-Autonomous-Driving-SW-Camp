import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2

#---TODO-------------------------------------
CAM_NUM = 0 
SHOW_IMAGE = True
TIMER = 0.1
QUE = 1
#--------------------------------------------

class CameraPublisherNode(Node):
    def __init__(self, cam_num=CAM_NUM, pub_topic='topic_raw_img', logger=SHOW_IMAGE, timer=TIMER, que=QUE):
        super().__init__('node_pub_cam')
        
        self.declare_parameter('cam_num', cam_num)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('logger', logger)
        self.declare_parameter('timer', timer)
        self.declare_parameter('que', que)
        
        self.cam_num = self.get_parameter('cam_num').get_parameter_value().integer_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer').get_parameter_value().double_value
        self.que = self.get_parameter('que').get_parameter_value().integer_value
           
        self.publisher = self.create_publisher(Image, self.pub_topic, self.que)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(self.cam_num)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.br = CvBridge()
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.publisher.publish(self.br.cv2_to_imgmsg(frame))
            
            if self.logger:
                cv2.imshow('cam pub',frame)
                cv2.waitKey(1)
      
def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
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
