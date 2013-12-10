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
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <PixelTriangulator.h>

#include <vector>

namespace image2pcl {

PixelTriangulator::PixelTriangulator(
    double pWidthCm) 
  : pixelWidthCm(pWidthCm) {
    ROS_INFO("Created a PixelTriangulator");

}

const pcl::PointCloud<pcl::PointXYZ> & PixelTriangulator::triangulate(
    pcl::PointCloud<pcl::PointXYZ> & cloud,
    const std::vector<PixelCorrespondence> & correspondences,
    const tf::Vector3 & lastPosition,
    const tf::Vector3 & currPosition,
    const tf::Quaternion & lastOrientation,
    const tf::Quaternion & currOrientation) {

  return cloud;
}

}  // namespace image2pcl


