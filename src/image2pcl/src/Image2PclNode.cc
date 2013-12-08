/*
 * The MIT License
 *
 * Copyright (c) 2013 Nicholas Wertzberger wertnick@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

#include <Image2PclNode.h>
#include <CloudPointBase.h>

using std::string;

namespace image2pcl {

/**
 * Base initializer for Image2PclNode
 */
Image2PclNode::Image2PclNode(
    ImageConverter & conv,
    CloudPointBase<cv::Vec3b> & base)
  : converter(conv), pointBase(base) {

  ros::NodeHandle nodeHandle;

  image_transport::ImageTransport imageTransport(nodeHandle);

  string image_topic = nodeHandle.resolveName("image");

  // Integrations with ROS
  cameraSubscriber = imageTransport.subscribeCamera(
      image_topic,
      1,
      &Image2PclNode::imageCb,
      this);

  ROS_INFO("Setting up publisher");

  imageStream = imageTransport.advertise("image_out", 1);
  pointCloudStream = nodeHandle
      .advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
}

void Image2PclNode::imageCb(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg) {
  cv_bridge::CvImagePtr bridge;
  try {
    bridge = cv_bridge::toCvCopy(
        image_msg,
        sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception & ex) {
    ROS_ERROR("[draw_frames] failed to convert image");
    return;
  }

  camModel.fromCameraInfo(info_msg);

  ROS_INFO("Converting Image");
  bridge->image = converter.convertImage(bridge->image);

  ROS_INFO("Publishing Converted Image");
  imageStream.publish(bridge->toImageMsg());

  ROS_INFO("Updating Point Cloud");
  pointBase.updateCloud(
      bridge->image, 
      tf::Vector3(0,0,0),
      tf::Quaternion(0,0,0,0),
      ros::Time()
  );

  ROS_INFO("Publishing Point Cloud");
  pointCloudStream.publish(pointBase.getCloud());
  
}

}   // namespace image2pcl
