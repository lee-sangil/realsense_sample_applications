// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

/////////////////////////////////////////////////////
// librealsense tutorial #1 - Accessing depth data //
/////////////////////////////////////////////////////

// First include the librealsense C++ header file
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <librealsense/rs.hpp>
#include <iostream>


int main() try
{
    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;


    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());


    // Configure depth to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
    dev->start();


    // Determine depth value corresponding to one meter (convert from [mm] to [m])
    const uint16_t one_meter = static_cast<uint16_t>(1.0f / dev->get_depth_scale());
    const double depth_scale = dev->get_depth_scale();


    while(true)
    {
        // This call waits until a new coherent set of frames is available on a device
        // Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        dev->wait_for_frames();


        // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
        const void * depth_frame = dev->get_frame_data(rs::stream::depth);
        cv::Mat depth_raw(480, 640, CV_16UC1, (void*)depth_frame);  //  unit : [mm]

        cv::Mat depth_in_meter;                                     //  unit : [m]
        depth_raw.convertTo(depth_in_meter, CV_64FC1);
        depth_in_meter = depth_in_meter * depth_scale;


        // print current depth value
        std::cout << depth_raw.at<unsigned short>(240,320) << std::endl;
        std::cout << depth_in_meter.at<double>(240,320) << std::endl;


        // depth image
        cv::imshow("depth_raw", depth_raw);
        cv::waitKey(1);
    }

    return EXIT_SUCCESS;
}


catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}
