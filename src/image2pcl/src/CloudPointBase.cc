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

#include <opencv/cv.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <CloudPointBase.h>

namespace image2pcl {

template<class T>
CloudPointBase<T>::CloudPointBase(PixelCorresponder<T> & corr)
  : cloud(new sensor_msgs::PointCloud2()),
    cloudPtr(cloud),
    corresponder(corr) {
}


template<class T>
void CloudPointBase<T>::updateCloud(
    const cv::Mat & image,
    const tf::Vector3 & position,
    const tf::Quaternion & orientation,
    const ros::Time & time) {
}

/**
 * get current "visible" cloud.
 */
template<class T>
const sensor_msgs::PointCloud2ConstPtr & CloudPointBase<T>::getCloud() {
  return cloudPtr;
}

template class CloudPointBase<cv::Vec3b>;
template class CloudPointBase<uint8_t>;
template class CloudPointBase<double>;

}   // namespace image2pcl
