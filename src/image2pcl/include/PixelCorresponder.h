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
#ifndef SRC_IMAGE2PCL_INCLUDE_PIXELCORRESPONDER_H_
#define SRC_IMAGE2PCL_INCLUDE_PIXELCORRESPONDER_H_

#include <opencv2/core/core.hpp>

#include <Macros.h>

#include <vector>

namespace image2pcl {

class PixelCorrespondence {
 public:
  PixelCorrespondence(cv::Point p1, cv::Point p2) : pixel1(p1), pixel2(p2) { }
  inline bool operator == (const PixelCorrespondence & p) const {
    return pixel1 == p.pixel1 && pixel2 == p.pixel2;
  }

  cv::Point pixel1;
  cv::Point pixel2;
  
};

/**
 * This is the tool to manage the visible point cloud.
 * A point remains visible so long as the last update determined it
 * to be visible in the image.
 */
template <class T>
class PixelCorresponder {
 public:
  PixelCorresponder(T skipVal);
  const std::vector<PixelCorrespondence> &
      correspondPixels(
          const cv::Mat & image1,
          const cv::Mat & image2,
          const int & xRadius,
          const int & yRadius);

 private:
  std::vector<PixelCorrespondence> pixels;
  T skipValue;

  cv::Point findCorrespondingPixel(
      const T & value,
      const cv::Point & pos,
      const cv::Mat & image,
      const int & xRadius,
      const int & yRadius);

  DISALLOW_COPY_AND_ASSIGN(PixelCorresponder);
};

}   // namespace image2pcl

std::ostream & operator << (
    std::ostream & stream,
    const image2pcl::PixelCorrespondence & corr); 

#endif  // SRC_IMAGE2PCL_INCLUDE_PIXELCORRESPONDER_H_
