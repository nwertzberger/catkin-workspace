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
#include <gtest/gtest.h>
#include <opencv/cv.h>
#include <PixelCorresponder.h>

#include <boost/foreach.hpp>

#include <vector>
#include <iostream>

using cv::Mat;
using cv::Mat_;

#define MAT(r, c, args...) (Mat_<int>(r, c) << args)

namespace image2pcl {

static std::vector<PixelCorrespondence> callCorrespondences(
    const Mat & leftEye,
    const Mat & rightEye) {
  return PixelCorresponder<int>(0).correspondPixels(leftEye, rightEye, 2, 2);
}

TEST(PixelCorresponder, instantiatesCleanly) {
  PixelCorresponder<cv::Vec3b> corresponder(0);
}

TEST(PixelCorresponder, canCorrespondASinglePixel) {
  std::vector<PixelCorrespondence> correspondences = callCorrespondences(
          MAT(3,4,
            0, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 0
          ),
          MAT(3,4,
            0, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 0
          ));

  ASSERT_EQ(1, correspondences.size());
  ASSERT_EQ(PixelCorrespondence(
          cv::Point(2,1),
          cv::Point(1,1)
     ), correspondences[0]);
}

TEST(PixelCorresponder, canCorrespondTwoPixels) {
  std::vector<PixelCorrespondence> correspondences = callCorrespondences(
          MAT(3,4,
            0, 0, 1, 0,
            0, 0, 1, 0,
            0, 0, 0, 0
          ),
          MAT(3,4,
            0, 1, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 0
          ));

  ASSERT_EQ(2, correspondences.size());
  ASSERT_EQ(PixelCorrespondence(
          cv::Point(2,0),
          cv::Point(1,0)
     ), correspondences[0]);
  ASSERT_EQ(PixelCorrespondence(
          cv::Point(2,1),
          cv::Point(1,1)
     ), correspondences[1]);
}

}   // namespace image2pcl

