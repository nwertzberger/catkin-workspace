#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

class FrameDrawer {
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport;
    image_transport::CameraSubscriber cameraSubscriber;
    image_transport::Publisher publisher;
    tf::TransformListener tfListener;
    image_geometry::PinholeCameraModel camModel;
    std::vector<std::string> frameIds;
    CvFont font;

public:

    /**
     * Base initializer for FrameDrawer
     */
    FrameDrawer(const std::vector<std::string> & frame_ids)
        : imageTransport(nodeHandle), frameIds(frame_ids)
    {
        std::string image_topic = nodeHandle.resolveName("image");
        cameraSubscriber = imageTransport.subscribeCamera(
                image_topic,
                1, 
                &FrameDrawer::imageCb,
                this
                );
        ROS_DEBUG("Setting up publisher");
        publisher = imageTransport.advertise("image_out", 1);
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    }

    void imageCb (  const sensor_msgs::ImageConstPtr& image_msg,
            const sensor_msgs::CameraInfoConstPtr& info_msg) {
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

        cv::Mat copy;
        cv::cvtColor(bridge->image, copy, CV_BGR2GRAY);
        cv::equalizeHist(copy, bridge->image);

        cv::cvtColor(bridge->image, bridge->image, CV_GRAY2BGR);

        // Publish
        publisher.publish(bridge->toImageMsg());
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "draw_frames");
    ROS_DEBUG("JUST STARTED");
    std::vector<std::string> frame_ids(argv + 1, argv + argc);
    ROS_DEBUG("STARTING UP FRAME DRAWER");
    FrameDrawer drawer (frame_ids);
    ROS_DEBUG("SPINNING");
    ros::spin();
}

