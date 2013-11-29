#include <ros/ros.h>
#include <Image2PclNode.hpp>

using namespace image2pcl;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "draw_frames");

    ROS_DEBUG("JUST STARTED");
    ROS_DEBUG("STARTING UP FRAME DRAWER");

    Image2PclNode drawer; 

    ROS_DEBUG("SPINNING");
    ros::spin();
}

