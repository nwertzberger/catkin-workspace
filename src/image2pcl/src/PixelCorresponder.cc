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
#include <math.h>

#include <boost/foreach.hpp>

#include <PixelCorresponder.h>
#include <iostream>

std::ostream & operator << (
    std::ostream & stream,
    const image2pcl::PixelCorrespondence & corr) {
  stream << "{p1:{" << corr.pixel1 << "}, p2:{" << corr.pixel2 <<"}}";
  return stream;
}

namespace image2pcl {

template <class T>
PixelCorresponder<T>::PixelCorresponder(T skipVal)
  : skipValue(skipVal) {
}

template <class T>
cv::Point PixelCorresponder<T>::findCorrespondingPixel(
    const T & value,
    const cv::Point & pos,
    const cv::Mat & image,
    const int & xRadius,
    const int & yRadius) {

  for (int x = pos.x - xRadius; x < pos.x + xRadius; x++) {
    for (int y = pos.y - yRadius; y < pos.y + yRadius; y++) {
      if (x >= 0 && y >= 0 && x < image.cols && y < image.rows 
          && image.at<T>(y,x) == value) {
        return cv::Point(x,y);
      }  
    }
  }
  return cv::Point(-1,-1);
}


/**
 *  Add the current information to the cloud database. The correspondence
 *  algorithm for this is to find every single pixel with a non-zero value, 
 *  correspond it to a pixel within a specified radius, and repeat.
 *  image : a processed image ready for correspondence.
 *
 *  Correspondence works as follows:
 *    Given xRadius, yRadius, rotation:
 *      Calculate correspondence cost to all points in space.
 *        cost = distance + adjacent pixel distance.  # may need to be tweaked.
 *      Select lowest cost correspondence.
 *          
 */
template <class T>
const std::vector<PixelCorrespondence> & PixelCorresponder<T>::correspondPixels(
    const cv::Mat & image1,
    const cv::Mat & image2,
    const int & xRadius,
    const int & yRadius) {

  pixels.clear();

  for (int x = 0; x < image1.rows; x++) {
    for (int y = 0; y < image1.cols; y++) {

      const cv::Point pixel1Pos(x, y);
      const T & value = image1.at<T>(pixel1Pos);

      // Skip values
      if (value == skipValue) {
        continue;
      }


      // find correspondences
      cv::Point pixel2Pos = findCorrespondingPixel(
          value,
          pixel1Pos,
          image2,
          xRadius,
          yRadius);
        
      BOOST_FOREACH(PixelCorrespondence c, pixels) {
        std::cout << c << ", ";
      }
      std::cout << "!" << std::endl;

      if (pixel2Pos.x != -1) {
        pixels.push_back(PixelCorrespondence(
            pixel1Pos,
            pixel2Pos
        ));
      }
    }
  }
  return pixels;
}

template class PixelCorresponder<cv::Vec3b>;
template class PixelCorresponder<uint8_t>;
template class PixelCorresponder<double>;

}  // namespace image2pcl


