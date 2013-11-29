#ifndef Image2PclNode_hpp
#define Image2PclNode_hpp
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/cv.h>

namespace image2pcl {

    /**
     * This node is the driver node. It configures the dependent nodes beneath it.
     */
    class Image2PclNode {
        image_transport::CameraSubscriber  cameraSubscriber;
        image_transport::Publisher         publisher;
        image_geometry::PinholeCameraModel camModel;

        cv::Ptr<CvFont> font;
        cv::Ptr<cv::FeatureDetector> detector;

    public:
        /**
         * Base initializer for Image2PclNode
         */
        Image2PclNode();
        void imageCb (
                const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg
                );
    };
}

#endif
