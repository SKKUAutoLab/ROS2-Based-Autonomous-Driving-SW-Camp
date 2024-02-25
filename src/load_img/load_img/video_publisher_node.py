import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import sys
import cv2
from load_img.lib import get_path

#---TODO-------------------------------------
VIDEO_PATH = get_path("2_record.mp4") # 현재 해당 경로에 비디오 파일이 없음
REPEAT = False
SHOW_IMAGE = True
SPEED = 10
QUE = 1
#--------------------------------------------

class VideoPublisherNode(Node):
    def __init__(self, video_path=VIDEO_PATH, repeat=REPEAT, pub_topic='topic_raw_img', logger=SHOW_IMAGE, speed=SPEED, que=QUE):
        super().__init__('node_pub_video')
        
        self.declare_parameter('video_path', video_path)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('repeat', repeat)
        self.declare_parameter('logger', logger)
        self.declare_parameter('speed', speed)
        self.declare_parameter('que', que)
        
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.repeat = self.get_parameter('repeat').get_parameter_value().bool_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.speed = self.get_parameter('speed').get_parameter_value().integer_value
        self.que = self.get_parameter('que').get_parameter_value().integer_value
        
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(self.video_path)
        
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        speed = (1/ fps) / self.speed
        self.get_logger().info("fps: "+str(fps))
        
        self.publisher = self.create_publisher(Image, self.pub_topic, self.que)
        
        if self.repeat:
            self.timer = self.create_timer( speed , self.repeat_callback)
        else:
            self.timer = self.create_timer( speed , self.timer_callback)

    def repeat_callback(self):
        while(self.cap.isOpened()):
            ret, frame = self.cap.read() 
            if ret:
                self.publisher.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
                
                if self.logger:
                    cv2.imshow('video frame', frame)
                    cv2.waitKey(1)
                          
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self.timer.cancel()
        self.cap.release()
        cv2.destroyAllWindows()
        sys.exit(0)              
            
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:            
            try:
                resized_img = cv2.resize(frame, (640, 480))
                msg = self.br.cv2_to_imgmsg(resized_img, encoding="bgr8")
                msg.header.frame_id = "video"
                self.publisher.publish(msg)
                
                if self.logger:
                    cv2.imshow('video frame', resized_img)
                    cv2.waitKey(1)
                else:
                    pass
            except CvBridgeError as e:
                print(e)

        else:
            print("\n\nvideo end")
            self.timer.cancel()
            self.cap.release()
            cv2.destroyAllWindows()
            sys.exit(0)  
      
def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
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
