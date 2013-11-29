#include <opencv/cv.h>
#include <opencv2/photo/photo.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/foreach.hpp>

#include <ImageConverter.hpp>

using namespace image2pcl;
using namespace std;

ImageConverter::ImageConverter() {
    detector = new cv::GoodFeaturesToTrackDetector(500, 0.07, 9., 3, false, 0.04);

    // Set up fonts
    font = new CvFont();
    cvInitFont(font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
}

const cv::Mat & ImageConverter::convertImage(cv::Mat & srcImage) {

    cv::Mat copy, blurred;
    cv::cvtColor(srcImage, srcImage, CV_BGR2GRAY);
    cv::blur(srcImage, blurred, cv::Size(7,7));
    cv::Canny(blurred, copy, 100, 200, 5);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(blurred, keypoints);

    //-- Draw keypoints
    cv::drawKeypoints(
            copy,
            keypoints,
            srcImage,
            cv::Scalar::all(-1),
            cv::DrawMatchesFlags::DEFAULT );

    return srcImage;
}

