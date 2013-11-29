#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <CloudPointBase.hpp>

using namespace image2pcl;
using namespace std;

CloudPointBase::CloudPointBase() {
}

/**
 *  Add the current information to the cloud database. The correspondence
 *  algorithm for this is to find every single pixel with a non-zero value, 
 *  correspond it to a pixel within a specified radius, and repeat.
 *  image : a processed image ready for correspondence.
 */
const vector<PixelCorrespondence> * CloudPointBase::correspondPixels(
        const cv::Mat & image1,
        const cv::Mat & image2,
        const int & xRadius,
        const int & yRadius,
        const double & orientation
        ) {
    return new vector<PixelCorrespondence>();
}


void CloudPointBase::updateCloud(
        const cv::Mat & image,
        const tf::Vector3 & position,
        const tf::Quaternion & orientation,
        const ros::Time & time
        ) {

}

/**
 * get current "visible" cloud.
 */
sensor_msgs::PointCloud2ConstPtr CloudPointBase::getCloud() {
    sensor_msgs::PointCloud2ConstPtr cloudPtr(cloud);
    return cloudPtr;
}

