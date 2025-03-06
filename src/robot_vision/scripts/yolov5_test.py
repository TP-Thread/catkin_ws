#!/usr/bin/python3
import rospy
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_vision.msg import BoundingBox

class ObjectDetector:
    def __init__(self, yolov5_path, weight_path, conf, sub_image_topic):
        # 使用pytorch加载yolov5模型，torch.hub.load会从robot_vision/yolov5/中找名为hubconf.py的文件
        # hubconf.py文件包含了模型的加载代码，负责指定加载哪个模型
        self.model = torch.hub.load(yolov5_path, 'custom', path=weight_path, source='local')
        self.model.conf = conf

        # 创建cv_bridge，声明图像的订阅者
        self.cv_bridge = CvBridge()
        self.image_sub = rospy.Subscriber(sub_image_topic, Image, self.callback)
        # 发布识别到的目标框信息
        self.target_pub = rospy.Publisher("/yolo_detections", BoundingBox, queue_size=1) 

    def callback(self, ros_image):
        # 将ROS的图像数据转换成OpenCV的图像格式            
        cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image, "bgr8")
        # 将Opencv图像转换numpy数组形式，数据类型是uint8（0~255），numpy提供了大量的操作数组的函数，可以方便高效地进行图像处理
        frame = np.array(cv_image, dtype=np.uint8)
        
        # 实例化BoundingBox，存储本次识别到的目标信息
        bounding_box = BoundingBox()

        # 将BGR图像转换为RGB图像, 给yolov5，其返回识别到的目标信息
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(rgb_image)
        boxs = results.pandas().xyxy[0].values

        # 筛选出置信度最大的AprilTag
        for box in boxs:
            temp = np.float64(box[4])
            bounding_box.confidence = 0

            if bounding_box.confidence < temp:                
                #（xmin, ymin）是目标的左上角，（xmax,ymax）是目标的右下角
                bounding_box.xmin = np.int64(box[0])
                bounding_box.ymin = np.int64(box[1])
                bounding_box.xmax = np.int64(box[2])
                bounding_box.ymax = np.int64(box[3])
                # 置信度，因为是基于统计，因此每个目标都有一个置信度，标识可能性
                bounding_box.confidence = temp
                # box[-1]是目标的类型名，比如person
                bounding_box.Class = box[-1]

        # 发布目标框信息
        self.target_pub.publish(bounding_box)


def main():
    rospy.init_node("yolov5_detector")
    rospy.loginfo("starting yolov5_detector node")

    # 指定yolov5的源码路径，位于robot_vision/yolov5/
    yolov5_path = rospy.get_param('~yolov5_path', '')
    # 指定yolov5的权重文件路径
    weight_path = rospy.get_param('~weight_path', '')
    # yolov5的模型置信度阈值，置信度低于conf的预测结果会被忽略
    conf = rospy.get_param('~conf', '0.5')
    # 订阅的图像话题
    sub_image_topic = rospy.get_param('~image_sub', '')
    
    yolov5_detector = ObjectDetector(yolov5_path, weight_path, conf, sub_image_topic)
    
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
