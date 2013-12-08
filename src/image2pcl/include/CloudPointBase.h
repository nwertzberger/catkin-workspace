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
#ifndef SRC_IMAGE2PCL_INCLUDE_CLOUDPOINTBASE_H_
#define SRC_IMAGE2PCL_INCLUDE_CLOUDPOINTBASE_H_

#include <opencv2/core/core.hpp>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Macros.h>
#include <PixelCorresponder.h>
#include <PixelTriangulator.h>

#include <vector>

namespace image2pcl {

/**
 * This is the tool to manage the visible point cloud.
 * A point remains visible so long as the last update determined it
 * to be visible in the image.
 */
template <class T>
class CloudPointBase {
 public:
  CloudPointBase(
      PixelCorresponder<T> & corr,
      PixelTriangulator    & trig);

  void updateCloud(
      const cv::Mat &         image,
      const tf::Vector3 &     position,
      const tf::Quaternion &  orientation,
      const ros::Time &       time);

  const sensor_msgs::PointCloud2ConstPtr & getCloud();

 private:
  PixelCorresponder<T> &  corresponder;
  PixelTriangulator &     triangulator;

  sensor_msgs::PointCloud2ConstPtr cloudPtr;

  cv::Mat         lastImage;
  tf::Vector3     lastPosition;
  tf::Quaternion  lastOrientation;
  ros::Time       lastTime;

  DISALLOW_COPY_AND_ASSIGN(CloudPointBase);
};

}   // namespace image2pcl

#endif  // SRC_IMAGE2PCL_INCLUDE_CLOUDPOINTBASE_H_
