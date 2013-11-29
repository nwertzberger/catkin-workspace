#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/photo/photo.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

#include "Image2PclNode.hpp"

using namespace image2pcl;
using namespace std;

/**
 * Base initializer for Image2PclNode
 */
Image2PclNode::Image2PclNode()
{
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport(nodeHandle);
    
    detector = new cv::GoodFeaturesToTrackDetector(500, 0.07, 9., 3, false, 0.04);
    string image_topic = nodeHandle.resolveName("image");

    // Integrations with ROS
    cameraSubscriber = imageTransport.subscribeCamera(
            image_topic,
            1, 
            &Image2PclNode::imageCb,
            this
        );
    ROS_DEBUG("Setting up publisher");
    publisher = imageTransport.advertise("image_out", 1);

    // Set up fonts
    font = new CvFont();
    cvInitFont(font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
}

void Image2PclNode::imageCb (
        const sensor_msgs::ImageConstPtr& image_msg,
        const sensor_msgs::CameraInfoConstPtr& info_msg
        ) {
    cv_bridge::CvImagePtr bridge;
    try {
        bridge = cv_bridge::toCvCopy(
                image_msg,
                sensor_msgs::image_encodings::BGR8
                );
    }
    catch (cv_bridge::Exception& ex) {
        ROS_ERROR("[draw_frames] failed to convert image");
        return;
    }

    camModel.fromCameraInfo(info_msg);

    std::vector<
    cv::Mat copy, blurred;
    cv::cvtColor(bridge->image, bridge->image, CV_BGR2GRAY);

    cv::blur(bridge->image, blurred, cv::Size(7,7));

    cv::Canny(blurred, copy, 100, 200, 5);

    std::vector<cv::KeyPoint> keypoints;

    detector->detect(blurred, keypoints);

    //-- Draw keypoints
    cv::drawKeypoints(
            copy,
            keypoints,
            bridge->image,
            cv::Scalar::all(-1),
            cv::DrawMatchesFlags::DEFAULT );

    // Publish
    publisher.publish(bridge->toImageMsg());
}
