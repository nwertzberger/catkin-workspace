#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

#include <Image2PclNode.hpp>

using namespace image2pcl;
using namespace std;

/**
 * Base initializer for Image2PclNode
 */
Image2PclNode::Image2PclNode()
{
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport(nodeHandle);
    
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
    bridge->image = converter.convertImage(bridge->image);

    // Publish
    publisher.publish(bridge->toImageMsg());
}
