#! /usr/bin/env python3
import rospy
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_vision.msg import BoundingBox, BoundingBoxes

class ObjectDetector:
    def __init__(self, yolov5_path, weight_path, conf, sub_image_topic):
        # 使用pytorch加载yolov5模型，torch.hub.load会从robot_vision/yolov5/中找名为hubconf.py的文件
        # hubconf.py文件包含了模型的加载代码，负责指定加载哪个模型
        self.model = torch.hub.load(yolov5_path, 'custom', path=weight_path, source='local')
        self.model.conf = conf

        # 创建cv_bridge，声明图像的发布者和订阅者
        self.cv_bridge = CvBridge()
        self.image_sub = rospy.Subscriber(sub_image_topic, Image, self.callback)
        # self.image_pub = rospy.Publisher("/usb_cam/image_yolo", Image, queue_size=1)
        self.image_pub = rospy.Publisher("/iris/camera/rgb/image_yolo", Image, queue_size=1)
        # 发布识别到的目标框信息，BoundingBoxes是自定义的消息类型
        self.target_pub = rospy.Publisher("/yolo_detections", BoundingBox, queue_size=1) 

    def callback(self, ros_image):
        # 将ROS的图像数据转换成OpenCV的图像格式            
        cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image, "bgr8")
        # 将Opencv图像转换numpy数组形式，数据类型是uint8（0~255），numpy提供了大量的操作数组的函数，可以方便高效地进行图像处理
        frame = np.array(cv_image, dtype=np.uint8)
        
        # 实例化BoundingBoxes，存储本次识别到的所有目标信息
        # bounding_boxes = BoundingBoxes()
        # bounding_boxes.header = ros_image.header
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

            # 放入box队列中
            # bounding_boxes.bounding_boxes.append(bounding_box)
        
        # 用绿框把目标圈出来
        # cv2.rectangle(cv_image, (bounding_box.xmin, bounding_box.ymin), (bounding_box.xmax, bounding_box.ymax), (0, 255, 0))    
        # 在框左上角打印物体类型信息Class  
        # cv2.putText(cv_image, bounding_box.Class, (bounding_box.xmin, bounding_box.ymin), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))    

        # 创建一个全黑的掩码
        mask = np.zeros_like(cv_image)
        # 只在 bounding_box 区域保留原图像
        mask[bounding_box.ymin:bounding_box.ymax, bounding_box.xmin:bounding_box.xmax] = \
            cv_image[bounding_box.ymin:bounding_box.ymax, bounding_box.xmin:bounding_box.xmax]
        # 更新 cv_image，使得框外区域变黑
        cv_image = mask
        cv_image = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
        # 替换为原始image的header，否则会出现无法同步的的问题。
        cv_image.header = ros_image.header

        # 将标识了识别目标的图像转换成ROS消息并发布
        self.image_pub.publish(cv_image)
        # 发布目标框的数据信息
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
