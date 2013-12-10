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

#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_datatypes.h>
#include <ros/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <CloudPointBase.h>

namespace image2pcl {

template<class T>
CloudPointBase<T>::CloudPointBase(
    PixelCorresponder<T> & corr,
    PixelTriangulator & trig)
  : corresponder(corr),
    triangulator(trig) {
      ROS_INFO("Instantiated a CloudPointBase");
}


template<class T>
void CloudPointBase<T>::updateCloud(
    const cv::Mat & image,
    const tf::Vector3 & position,
    const tf::Quaternion & orientation,
    const ros::Time & time) {
  ROS_INFO("validating image size");
  // If there was nothing in the last image, there is nothing to do.
  if (lastImage.rows == 0 && lastImage.cols == 0) {
    ROS_INFO("Corresponding pixels");
    const std::vector<PixelCorrespondence> & corr = corresponder.correspondPixels(
        lastImage,
        image,
        3, 3);

    ROS_INFO("Triangulating correspondences");
    triangulator.triangulate(
        cloud,
        corr,
        lastPosition,
        position,
        lastOrientation,
        orientation);
  }

  lastImage = image;
  lastPosition = position;
  lastOrientation = orientation;
  lastTime = time;
}

/**
 * get current "visible" cloud.
 */
template<class T>
const pcl::PointCloud<pcl::PointXYZ> & CloudPointBase<T>::getCloud() {
  return cloud;
}

template<class T>
bool CloudPointBase<T>::hasCloud() {
  return cloud.empty();
}

template class CloudPointBase<cv::Vec3b>;
template class CloudPointBase<uint8_t>;
template class CloudPointBase<double>;

}   // namespace image2pcl
