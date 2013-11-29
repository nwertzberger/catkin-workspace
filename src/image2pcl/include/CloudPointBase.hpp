#ifndef ImageConverter_hpp
#define ImageConverter_hpp

#include <opencv/cv.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

namespace image2pcl {

    class PixelCorrespondence {
        cv::Point pixel1;
        cv::Point pixel2;
    }

    /**
     * This is the tool to manage the visible point cloud.
     * A point remains visible so long as the last update determined it
     * to be visible in the image.
     */
    class CloudPointBase {
        private:
            sensor_msgs::PointCloud2 cloud;
            const std::vector<PixelCorrespondence> &
                correspondPixels(
                        const cv::Mat & image1,
                        const cv::Mat & image2,
                        const int & xRadius,
                        const int & yRadius,
                        const double & orientation
                        ); 
        public:
            CloudPointBase();
            void updateCloud(
                    const cv::Mat & image,
                    const tf::Vector3 & position,
                    const tf::Quaternion & orientation,
                    const ros::Time & time
                    );

            sensor_msgs::PointCloud2ConstPtr getCloud();
    };
}

#endif
