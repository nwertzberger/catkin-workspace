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
#include <tf/transform_datatypes.h>

#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <CloudPointBase.h>

namespace image2pcl {

CloudPointBase::CloudPointBase() {
}

/**
 *  Add the current information to the cloud database. The correspondence
 *  algorithm for this is to find every single pixel with a non-zero value, 
 *  correspond it to a pixel within a specified radius, and repeat.
 *  image : a processed image ready for correspondence.
 */
const std::vector<PixelCorrespondence> & CloudPointBase::correspondPixels(
    const cv::Mat & image1,
    const cv::Mat & image2,
    const int & xRadius,
    const int & yRadius,
    const double & orientation) {
  return pixels;
}


void CloudPointBase::updateCloud(
    const cv::Mat & image,
    const tf::Vector3 & position,
    const tf::Quaternion & orientation,
    const ros::Time & time) {
}

/**
 * get current "visible" cloud.
 */
sensor_msgs::PointCloud2ConstPtr CloudPointBase::getCloud() {
  return sensor_msgs::PointCloud2ConstPtr(&cloud);
}

}   // namespace image2pcl
