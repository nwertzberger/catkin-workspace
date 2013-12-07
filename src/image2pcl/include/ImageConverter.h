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
#ifndef SRC_IMAGE2PCL_INCLUDE_IMAGECONVERTER_H_
#define SRC_IMAGE2PCL_INCLUDE_IMAGECONVERTER_H_

#include <opencv/cv.h>
#include <Macros.h>

namespace image2pcl {

/**
 * This is the actual converter. Its job is to prep an image for being
 * converted to a Point Cloud through cv filtering.
 */
class ImageConverter {
 public:
  ImageConverter();
  const cv::Mat & convertImage(cv::Mat & srcImage);
 private:
  DISALLOW_COPY_AND_ASSIGN(ImageConverter);
};

}   // namespace image2pcl

#endif  // SRC_IMAGE2PCL_INCLUDE_IMAGECONVERTER_H_
