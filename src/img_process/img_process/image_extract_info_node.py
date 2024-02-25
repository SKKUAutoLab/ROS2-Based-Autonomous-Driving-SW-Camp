import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from interfaces_pkg.msg import LaneInfo

from .lib import extract_road_info as RF

#---TODO-------------------------------------
SUB_TOPIC_NAME = "topic_postproc_img"
PUB_TOPIC_NAME = "topic_lane_info"

# PPT 자료 참고
lane_horizontal_thesh_degree = 18
lane_width_pixel = 300
target_point_detection_height = 90

TIMER = 0.1
QUE = 1
#--------------------------------------------

class ExtractInfo():
  def __init__(self):
    pass

  def process(self, img):

    '''
    주행 도로의 기울기, 주행 도로의 목표점의 좌표를 추출하는 노드

      ---------------------------------------------------------------------------
      <받는 데이터>                                                                  
        img : mask처리된 이미지  

      <변수>
        None

      * dominant_gradient, extract_road_center 함수를 활용하여 아래의 정보들을 publish
        * road_gradient : mask된 도로 영상에서의 gradient
        * road_target_point_x, road_target_point_y : 주행 도로의 목표점의 x,y 좌표
      ---------------------------------------------------------------------------
      
    '''
    ### Start (여기만 수정하세요)

    road_gradient = RF.dominant_gradient(img, angle_upper_limit=lane_horizontal_thesh_degree)
    print('road gradient: ', road_gradient)

    road_target_point_x, road_target_point_y = RF.extract_road_center(img, 
                                                    target_point_detection_height=target_point_detection_height, 
                                                    road_gradient=road_gradient, 
                                                    angle_upper_limit = lane_horizontal_thesh_degree, 
                                                    lane_width=lane_width_pixel) 
    print('road_target_point_x: ', road_target_point_x)
    print('road_target_point_y: ', road_target_point_y)

    ### END 
    
    return road_gradient, road_target_point_x, road_target_point_y

class ExtractInfoNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME, pub_topic=PUB_TOPIC_NAME, timer=TIMER, que=QUE):
    super().__init__('node_info_extraction')
    
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
    self.detect = ExtractInfo()
    
    image_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=self.que)
    self.subscription = self.create_subscription(Image, self.sub_topic, self.image_callback, image_qos_profile)

    self.publisher_ = self.create_publisher(LaneInfo, self.pub_topic , self.que)
    self.timer = self.create_timer(self.timer_period, self.timer_callback)

  def image_callback(self, data):
    self.is_running = True
    current_frame = self.br.imgmsg_to_cv2(data)
    
    grad, road_target_point_x, road_target_point_y = self.detect.process(current_frame)
    
    lane = LaneInfo()
    lane.slope = grad
    lane.target_x = round(road_target_point_x)
    lane.target_y = round(road_target_point_y)
    
    self.publisher_.publish(lane)

  def timer_callback(self):
    if not self.is_running:
      self.get_logger().info('Not published yet: "%s"' % self.sub_topic)
      
def main(args=None):
    rclpy.init(args=args)
    node = ExtractInfoNode()
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
