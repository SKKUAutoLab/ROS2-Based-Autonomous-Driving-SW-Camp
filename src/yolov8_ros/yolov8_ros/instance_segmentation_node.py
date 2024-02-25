from yolov8_ros import *

import cv2
import random
import numpy as np
from typing import List, Dict, Tuple
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

from sensor_msgs.msg import Image
from interfaces_pkg.msg import Point2D
from interfaces_pkg.msg import BoundingBox2D
from interfaces_pkg.msg import Mask
from interfaces_pkg.msg import Detection
from interfaces_pkg.msg import DetectionArray

#---TODO-------------------------------------
SUB_TOPIC_NAME = 'topic_raw_img'
PUB_TOPIC_NAME = 'topic_masking_img'
SHOW_IMAGE = True
TIMER = 0.1
QUE = 1

MODEL = get_path('best.pt') # lib 안에 위치한 pt 파일명
DEVICE = "cpu"
#DEVICE = "cuda:0"
THRESHOLD = 0.5 # 0.1
#--------------------------------------------

WHITE = [255, 255, 255]
PINK = [136, 0, 237]
PURPLE = [37, 0, 136]
            
class SegmentationNode(Node):
    def __init__(self, sub_topic=SUB_TOPIC_NAME, pub_topic=PUB_TOPIC_NAME, logger=SHOW_IMAGE, timer=TIMER, que=QUE, \
                        model=MODEL, device=DEVICE, threshold=THRESHOLD   \
        ):
        super().__init__("node_pub_segmentation")
        
        self.declare_parameter('sub_topic', sub_topic)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('logger', logger)
        self.declare_parameter('timer', timer)
        self.declare_parameter('que', que)
        
        self.declare_parameter("model", model)
        self.declare_parameter("device", device)
        self.declare_parameter("threshold", threshold)
                
        self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer').get_parameter_value().double_value
        self.que = self.get_parameter('que').get_parameter_value().integer_value
        
        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
               
        self.cv_bridge = CvBridge()
        image_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=self.que)
        
        self.model = get_path(self.model)
        self.yolo = YOLO(self.model)
        self.yolo.fuse()
        self.yolo.to(self.device)
        
        self.sub = self.create_subscription(Image, self.sub_topic, self.image_callback, image_qos_profile)
        
        self.pub_img = self.create_publisher(Image, self.pub_topic, self.que)
        self.pub_info = self.create_publisher(DetectionArray, "detections", self.que)
        
        self._class_to_color = {}
        
        self.time_prev = time.time()
        self.time_post = time.time()
            
    def parse_hypothesis(self, results: Results) -> List[Dict]:
        hypothesis_list = []
        box_data: Boxes
        for box_data in results.boxes:            
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis) 
        return hypothesis_list
    
    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:
        boxes_list = []
        box_data: Boxes
        for box_data in results.boxes:
            msg = BoundingBox2D()
            box = box_data.xywh[0]
            msg.center.x = float(box[0]) # x
            msg.center.y = float(box[1]) # y
            msg.size.x = float(box[2])   # w
            msg.size.y = float(box[3])   # h
            boxes_list.append(msg)
        return boxes_list
    
    def parse_masks(self, results: Results) -> List[Mask]:
        masks_list = []
        
        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p
        
        mask: Masks
        for mask in results.masks:
            msg = Mask()
            msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                        for ele in mask.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]
            masks_list.append(msg)
        return masks_list
    
    def masking_result(self, ori_img, mask_points, color):    
        mask_array = np.array([[int(ele.x), int(ele.y)] for ele in mask_points])
        res = []
        if mask_points:
            layer = ori_img
            layer = cv2.fillPoly(layer, pts=[mask_array], color=color)
            cv2.addWeighted(ori_img, 0.4, layer, 0.6, 0, ori_img)
            res = cv2.polylines(ori_img, [mask_array], isClosed=True, color=color, thickness=2, lineType=cv2.LINE_AA)
        return res
    
    def is_empty_img(self, ori_img, chk_img):
        if chk_img.size == 0 or chk_img.shape[0] == 0 or chk_img.shape[1] == 0:
            self.get_logger().warn("Received an empty image. Skipping processing.")
            chk_img = ori_img
            #cv2.imwrite( get_time(), chk_img)
        return chk_img
    
    def image_callback(self, msg: Image) -> None:
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        
        results = self.yolo.predict(source=cv_image, verbose=False, stream=False, conf=self.threshold, device=self.device)
        results: Results = results[0].cpu()
        
        if results.boxes:
            hypothesis = self.parse_hypothesis(results) # id, class name, score
            boxes = self.parse_boxes(results)           # box x, y, size
        
        if results.masks:
            masks = self.parse_masks(results)           # mask

        detections_msg = DetectionArray()
        
        copy_img = cv_image.copy()                 
        black_img = np.zeros_like(copy_img) 
        
        result_img = np.array([])
        
        for i in range(len(results)):
            aux_msg = Detection()
            if results.boxes:
                aux_msg.class_id = hypothesis[i]["class_id"]
                aux_msg.class_name = hypothesis[i]["class_name"]
                aux_msg.score = hypothesis[i]["score"]
                aux_msg.bbox = boxes[i]
            if results.masks:
                aux_msg.mask = masks[i]
            detections_msg.detections.append(aux_msg)
            

            label = aux_msg.class_name
            
            if label not in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[label] = (r, g, b)
            color = self._class_to_color[label]
            label = aux_msg.class_name
            
            
            if label == 'lane2':
                color = PINK
                result_img = self.masking_result(black_img, aux_msg.mask.data, color) # 영역 외부는 검정
                    
        masking_outer = self.is_empty_img(cv_image, result_img)
        
        detections_msg.header = msg.header
        self.pub_img.publish(self.cv_bridge.cv2_to_imgmsg(masking_outer, encoding=msg.encoding))
        self.pub_info.publish(detections_msg)
                                     
        cv2.imshow('masked_img', masking_outer)
        cv2.waitKey(1)

        self.time_prev = self.time_post
        self.time_post = time.time()
        print('computation time for this frame: ', self.time_post - self.time_prev)

        
def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
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