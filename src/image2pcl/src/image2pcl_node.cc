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
#include <ros/ros.h>
#include <opencv/cv.h>
#include <Image2PclNode.h>
#include <PixelCorresponder.h>
#include <PixelTriangulator.h>
#include <CloudPointBase.h>

using namespace image2pcl;

/**
 * This function is responsible for setting up the dependencies for the
 * different classes and then starting it up.
 */
int main(int argc, char ** argv) {
    ros::init(argc, argv, "draw_frames");

    ROS_DEBUG("JUST STARTED");
    ROS_DEBUG("STARTING UP FRAME DRAWER");

    // Set up the class dependencies
    // A Corresponder simple finds correspondences between two cv::Mat's
    PixelCorresponder<cv::Vec3b> corresponder(cv::Vec3b(0,0,0));

    // A Triangulator converts these correspondences into a point cloud
    PixelTriangulator triangulator(1);

    // A CloudPointBase manages stateful information.
    CloudPointBase<cv::Vec3b> pointBase(
        corresponder,
        triangulator);

    // An image converter is responsible for making the image ready to
    // process.
    ImageConverter    converter;

    // An Image2PclNode is responsible for hiding all the ROS configuration
    // needed to start up a nodehandle and publish the different messages.
    Image2PclNode drawer(converter, pointBase);

    ROS_DEBUG("SPINNING");
    ros::spin();
}

