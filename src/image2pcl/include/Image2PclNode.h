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
#ifndef SRC_IMAGE2PCL_INCLUDE_IMAGE2PCLNODE_H_
#define SRC_IMAGE2PCL_INCLUDE_IMAGE2PCLNODE_H_
#include <ros/publisher.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/PointCloud2.h>

#include <ImageConverter.h>
#include <CloudPointBase.h>
#include <Macros.h>

namespace image2pcl {

/**
 * This node is the driver node. It configures the dependent nodes beneath it.
 */
class Image2PclNode {
 public:
  Image2PclNode(
      ImageConverter & conv,
      CloudPointBase<cv::Vec3b> & base);
  void imageCb(
      const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::CameraInfoConstPtr& info_msg);

 private:
  ImageConverter &                  converter;
  CloudPointBase<cv::Vec3b> &       pointBase;

  image_transport::Publisher        imageStream;
  ros::Publisher                    pointCloudStream;

  sensor_msgs::PointCloud2          cloudMsg;

  image_transport::CameraSubscriber  cameraSubscriber;
  image_geometry::PinholeCameraModel camModel;

  DISALLOW_COPY_AND_ASSIGN(Image2PclNode);
};

}   // namespace image2pcl

#endif  // SRC_IMAGE2PCL_INCLUDE_IMAGE2PCLNODE_H_
