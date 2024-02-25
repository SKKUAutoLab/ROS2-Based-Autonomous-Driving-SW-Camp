import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2

from .lib import filter_image as FILTER

#---TODO-------------------------------------
#SUB_TOPIC_NAME = "topic_raw_img"
SUB_TOPIC_NAME = "topic_masking_img"
PUB_TOPIC_NAME = "topic_postproc_img"

TIMER = 0.1
QUE = 1
#--------------------------------------------

class PostProcess():
  def __init__(self):
    pass

  def process(self, img):
    '''
    도로의 정보를 추출하기전에 모델 추론 영상에 대한 처리를 하는 노드
    
      ---------------------------------------------------------------------------
      <받는 데이터>                                                                  
        img : deeplearning model 추론 결과 이미지

      <변수>
        result_img : 최종 return해줘야하는 변수명

      *bird_eye view, roi 변환 함수, OpenCV 제공하는 blur, edge 함수를 활용하여 자유롭게 알고리즘 작성하시오.
        ex. 추론 이미지 -> bird_eye view 변환 -> blur 함수 -> gray 변환 함수 -> edge함수 -> roi 변환 함수
        (1). 현재 처리되는 화면을 확인하고싶으면, cv2.imshow()함수를 사용한다.
        (2). (1)을 사용할 시 cv2.waitkey()함수는 필수로 사용한다.

      ---------------------------------------------------------------------------
   
    '''
    ### Start (여기만 수정하세요)

    (h, w) = (img.shape[0], img.shape[1])
    dst_mat = [[round(w * 0.3), round(h * 0.0)], [round(w * 0.7), round(h * 0.0)], [round(w * 0.7), h], [round(w * 0.3), h]]
    src_mat = [[238, 316],[402, 313], [501, 476], [155, 476]]

    bird_img = FILTER.bird_convert(img, srcmat=src_mat, dstmat=dst_mat) 
    ret,binart_image = cv2.threshold(bird_img, 0, 255, cv2.THRESH_BINARY)
    img_blur = cv2.GaussianBlur(binart_image, (3,3),2.38)
    img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)
    img_edge = cv2.Canny(img_gray, 110,180)
    roi_img = FILTER.roi_rectangle_below(img_edge, 300)
    
    result_img = roi_img
    cv2.imshow('postprocess_image', result_img)
    cv2.waitKey(1)
    ### END 
    
    return result_img

class ImgPostProcessNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME, pub_topic=PUB_TOPIC_NAME, timer=TIMER, que=QUE):
    super().__init__('node_pub_postproc')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('pub_topic', pub_topic)
    self.declare_parameter('timer', timer)
    self.declare_parameter('que', que)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
    self.timer_period = self.get_parameter('timer').get_parameter_value().double_value
    self.que = self.get_parameter('que').get_parameter_value().integer_value

    self.is_running = False
    self.br = CvBridge()
    self.post = PostProcess()
    
    image_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=self.que)
    self.subscription = self.create_subscription(Image, self.sub_topic, self.image_callback, image_qos_profile)

    self.publisher_ = self.create_publisher(Image, self.pub_topic , self.que)
    self.timer = self.create_timer(self.timer_period, self.timer_callback)

  def image_callback(self, data):
    self.is_running = True
    current_frame = self.br.imgmsg_to_cv2(data)
    processed_img = self.post.process(current_frame)
    self.publisher_.publish(self.br.cv2_to_imgmsg(processed_img))

  def timer_callback(self):
    if not self.is_running:
      self.get_logger().info('Not published yet: "%s"' % self.sub_topic)

def main(args=None):
    rclpy.init(args=args)
    node = ImgPostProcessNode()
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
