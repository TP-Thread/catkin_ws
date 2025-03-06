/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
  void ContinuousDetector::onInit()
  {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &pnh = getPrivateNodeHandle();

    tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
    draw_tag_detections_image_ = getAprilTagOption<bool>(pnh,
                                                         "publish_tag_detections_image", false);
    it_ = std::shared_ptr<image_transport::ImageTransport>(
        new image_transport::ImageTransport(nh));

    std::string transport_hint;
    pnh.param<std::string>("transport_hint", transport_hint, "raw");

    int queue_size;
    pnh.param<int>("queue_size", queue_size, 1);
    camera_image_subscriber_ =
        it_->subscribeCamera("image_rect", queue_size,
                             &ContinuousDetector::imageCallback, this,
                             image_transport::TransportHints(transport_hint));
    tag_detections_publisher_ =
        nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
    if (draw_tag_detections_image_)
    {
      tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
    }

    refresh_params_service_ =
        pnh.advertiseService("refresh_tag_params",
                             &ContinuousDetector::refreshParamsCallback, this);

    yolo_subscriber_ = nh.subscribe("/yolo_detections", 1, &ContinuousDetector::yoloCallback, this);
  }

  void ContinuousDetector::refreshTagParameters()
  {
    // Resetting the tag detector will cause a new param server lookup
    // So if the parameters have changed (by someone/something),
    // they will be updated dynamically
    std::scoped_lock<std::mutex> lock(detection_mutex_);
    ros::NodeHandle &pnh = getPrivateNodeHandle();
    tag_detector_.reset(new TagDetector(pnh));
  }

  bool ContinuousDetector::refreshParamsCallback(std_srvs::Empty::Request &req,
                                                 std_srvs::Empty::Response &res)
  {
    refreshTagParameters();
    return true;
  }

  void ContinuousDetector::yoloCallback(const robot_vision::BoundingBox::ConstPtr &msg)
  {
    std::scoped_lock<std::mutex> lock(yolo_mutex_);

    // 检查置信度阈值
    if (msg->confidence > 0.5)
    {
      yolo_bbox_[0] = msg->xmin;
      yolo_bbox_[1] = msg->ymin;
      yolo_bbox_[2] = msg->xmax;
      yolo_bbox_[3] = msg->ymax;
      yolo_bbox_[4] = msg->confidence;
    }
  }

  void ContinuousDetector::imageCallback(
      const sensor_msgs::ImageConstPtr &image_rect,
      const sensor_msgs::CameraInfoConstPtr &camera_info)
  {
    std::scoped_lock<std::mutex> lock(detection_mutex_);
    // Lazy updates:
    // When there are no subscribers _and_ when tf is not published,
    // skip detection.
    if (tag_detections_publisher_.getNumSubscribers() == 0 &&
        tag_detections_image_publisher_.getNumSubscribers() == 0 &&
        !tag_detector_->get_publish_tf())
    {
      // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
      return;
    }

    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTag 2 on the iamge
    try
    {
      cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // 创建一个全黑的掩码
    cv::Mat mask = cv::Mat::zeros(cv_image_->image.size(), cv_image_->image.type());

    // 如果检测到目标，目标区域保留，其余部分变黑
    if (yolo_bbox_[4] > 0.5)
    {
      cv::Rect roi(
          yolo_bbox_[0], yolo_bbox_[1],
          yolo_bbox_[2] - yolo_bbox_[0],
          yolo_bbox_[3] - yolo_bbox_[1]);

      // 确保 ROI 在图像范围内
      roi &= cv::Rect(0, 0, cv_image_->image.cols, cv_image_->image.rows);

      // 复制目标区域到掩码
      cv_image_->image(roi).copyTo(mask(roi));

      // 更新原始图像
      cv_image_->image = mask;
    }

    // Publish detected tags in the image by AprilTag 2
    tag_detections_publisher_.publish(
        tag_detector_->detectTags(cv_image_, camera_info));

    // Publish the camera image overlaid by outlines of the detected tags and
    // their payload values
    if (draw_tag_detections_image_)
    {
      tag_detector_->drawDetections(cv_image_);
      tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
  }

} // namespace apriltag_ros
