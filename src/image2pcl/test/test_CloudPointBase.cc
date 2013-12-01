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
#include <gtest/gtest.h>

#include <opencv/cv.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <PixelCorresponder.h>
#include <CloudPointBase.h>

namespace image2pcl {

TEST(CloudPointBaseTest, instantiatesCleanly) {
  PixelCorresponder corresponder;
  CloudPointBase pointBase(corresponder);
}

TEST(CloudPointBaseTest, canCreateASinglePointCloud) {
  PixelCorresponder corresponder;
  CloudPointBase pointBase(corresponder);

  uint8_t leftEyeData[3][4] = {
    {0, 0, 0, 0},
    {0, 0, 100, 0},
    {0, 0, 0, 0}
  };
  uint8_t rightEyeData[3][4] = {
    {0, 0, 0, 0},
    {0, 100, 0, 0},
    {0, 0, 0, 0}
  };

  cv::Mat leftEye(3, 4, CV_8U, leftEyeData);
  cv::Mat rightEye(3, 4, CV_8U, rightEyeData);

  tf::Vector3 leftPos(0, 0, 0);
  tf::Vector3 rightPos(1, 0, 0);

  tf::Quaternion orientation(0.0, 0.0, 0.0, 0.0);
  
  pointBase.updateCloud(
      leftEye,
      leftPos,
      orientation,
      ros::Time(0.0));

  pointBase.updateCloud(
      rightEye,
      rightPos,
      orientation,
      ros::Time(0.01));

  const sensor_msgs::PointCloud2ConstPtr & cloud = pointBase.getCloud();
      
}

}   // namespace image2pcl

