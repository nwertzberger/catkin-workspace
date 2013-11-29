#ifndef ImageConverter_hpp
#define ImageConverter_hpp

#include <opencv/cv.h>

namespace image2pcl {

    /**
     * This is the actual converter. Its job is to prep an image for being
     * converted to a Point Cloud through cv filtering.
     */
    class ImageConverter {
        cv::Ptr<CvFont> font;
        cv::Ptr<cv::FeatureDetector> detector;
    public:
        ImageConverter();
        const cv::Mat & convertImage(cv::Mat & srcImage);
    };
}

#endif
