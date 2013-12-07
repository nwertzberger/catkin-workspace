/*
 * The MIT License
 *
 * Copyright (c) 2013 Nicholas Wertzberger wertnick@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <opencv/cv.h>
#include <opencv2/photo/photo.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/foreach.hpp>

#include <ImageConverter.h>

namespace image2pcl {

ImageConverter::ImageConverter() {
}

const cv::Mat & ImageConverter::convertImage(cv::Mat & srcImage) {
    cv::Mat blurred;
    cv::cvtColor(srcImage, srcImage, CV_BGR2GRAY);
    cv::blur(srcImage, blurred, cv::Size(7, 7));
    cv::Canny(blurred, srcImage, 100, 200, 5);
    return srcImage;
}

}   // namespace image2pcl
