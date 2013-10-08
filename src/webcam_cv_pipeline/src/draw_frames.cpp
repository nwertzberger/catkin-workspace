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
        ROS_DEBUG("GOT A MESSAGE");
        cv::Mat image;
        cv_bridge::CvImagePtr input_bridge;
        try {
            input_bridge = cv_bridge::toCvCopy(
                image_msg,
                sensor_msgs::image_encodings::BGR8
            );
            image = input_bridge->image;
        }
        catch (cv_bridge::Exception& ex) {
            ROS_ERROR("[draw_frames] failed to convert image");
            return;
        }

        camModel.fromCameraInfo(info_msg);

        BOOST_FOREACH(const std::string& frame_id, frameIds) {
            tf::StampedTransform transform;
            try {
                ros::Time acquisition_time = info_msg->header.stamp;
                ros::Duration timeout(1.0/3.0);
                tfListener.waitForTransform(
                        camModel.tfFrame(),
                        frame_id,
                        acquisition_time,
                        timeout);
                tfListener.lookupTransform(
                        camModel.tfFrame(), 
                        frame_id,
                        acquisition_time,
                        transform);
            } catch (tf::TransformException& ex) {
                ROS_WARN("[draw_frames TF exception:\n%s", ex.what());
                return;
            }

            tf::Point pt = transform.getOrigin();
            cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
            cv::Point2d uv = camModel.project3dToPixel(pt_cv);

            static const int RADIUS = 3;
            cv::circle(image, uv, RADIUS, CV_RGB(255, 0, 0), -1);
            CvSize text_size;
            int baseline;
            cvGetTextSize(frame_id.c_str(), &font, &text_size, &baseline);
            CvPoint origin = cvPoint(
                uv.x - text_size.width / 2,
                uv.y - RADIUS - baseline - 3
            );
            cv::putText(
                image,
                frame_id.c_str(),
                origin,
                cv::FONT_HERSHEY_SIMPLEX,
                12,
                CV_RGB(255,0,0)
            );
        }
        ROS_DEBUG("Publising image message");
        publisher.publish(input_bridge->toImageMsg());
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

