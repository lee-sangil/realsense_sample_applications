// This program is handling intel realsense  R200 with infrared image.
// Note. This program surely is used on R200.

#include <iostream>
#include <cstring>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense/rs.hpp>

int main(int argc, char* argv[]) try
{
	bool isSave = false;
	if(argc != 3){
		std::cout << "Usage: ./librealsense --save [bool, default is false]" << std::endl;
	}
	else{
		if(!strcmp(argv[1], "--save"))
			isSave = argv[2];
		else
			std::cout << "Usage: ./librealsense --save [bool, default is false]" << std::endl;
	}

	// Create a context object. This object owns the handles to all connected realsense devices.
	rs::context ctx;
	std::cout << "There are " << ctx.get_device_count() << " connected RealSense device.\n" << std::endl;
	if(ctx.get_device_count() == 0)
	{
		return EXIT_FAILURE;
	}


	// This tutorial will access only a single device, but it is trivial to extend to multiple devices
	rs::device * dev = ctx.get_device(0);
	const double depth_scale = dev->get_depth_scale();

	std::cout << "\nUsing device 0, an " << dev->get_name() << std::endl;
	std::cout << "Serial number: " << dev->get_serial() << std::endl;
	std::cout << "Firmware version: " << dev->get_firmware_version() << std::endl;
	std::cout << "Depth scale: " << depth_scale << std::endl;

	std::string temp = (isSave == true)? "true": "false";
	std::cout << "Save: " << temp << std::endl << std::endl;

	// Apply depth preset
	rs::apply_depth_control_preset(dev, 5);

	// Configure depth to run at VGA resolution at 60 frames per second
	dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
	dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 60);
	dev->start();

	// Make folders for saving current image and depth
	std::string image_filename, depth_filename;
	std::string image_dir = "log/rgb/";
	std::string depth_dir = "log/depth/";
	std::string directory_create_command = "mkdir -p " + image_dir + " " + depth_dir;
	system(directory_create_command.c_str());

	// Make rgb and depth filename log
	std::ofstream rgb_log, depth_log;
	rgb_log.open("log/rgb.txt");
	depth_log.open("log/depth.txt");

	rgb_log << "# color images" << std::endl;
	rgb_log << "# file" << std::endl;
	rgb_log << "# timestamp filename" << std::endl;

	depth_log << "# depth images" << std::endl;
	depth_log << "# file" << std::endl;
	depth_log << "# timestamp filename" << std::endl;

	// Get intrinsics and extrinsics
	rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
	rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
	rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);

	std::cout.precision(10);
	std::cout << "# Intrinsic matrix" << std::endl;
	std::cout << "fx: " << color_intrin.fx << std::endl;
	std::cout << "fy: " << color_intrin.fy << std::endl;
	std::cout << "cx: " << color_intrin.ppx << std::endl;
	std::cout << "cy: " << color_intrin.ppy << std::endl;

	std::cout << "# extrinsic matrix" << std::endl;
	for(int i=0;i<3;i++)
		std::cout << "t[" << i << "]: " << depth_to_color.translation[i] << std::endl;

	for(int i=0;i<9;i++)
		std::cout << "r[" << i << "]: " << depth_to_color.rotation[i] << std::endl;

	while(1){
		std::cout << std::endl << "Continue?[y/n]";

		char input[2];
		std::cin >> input;

		if(!strcmp(input, "n"))
			return EXIT_SUCCESS;
		else if(!strcmp(input, "y"))
			break;
	}

	// Buffer for filename
	char filename[255];

	while(true)
	{
		dev->wait_for_frames();

		// Get depth and color frame and Change the format to Mat files
		const void * color_frame = dev->get_frame_data(rs::stream::rectified_color);
		const void * depth_aligned_frame = dev->get_frame_data(rs::stream::depth_aligned_to_rectified_color);
		const void * depth_raw_frame = dev->get_frame_data(rs::stream::depth);

		double timestamp = dev->get_frame_timestamp(rs::stream::color);

		cv::Mat color(480, 640, CV_8UC3, (void*)color_frame);
		cv::Mat depth_aligned(480, 640, CV_16UC1, (void*)depth_aligned_frame);
		cv::Mat depth_raw(480, 640, CV_16UC1, (void*)depth_raw_frame);
		cv::Mat depth_meter(480, 640, CV_16UC1, (void*)depth_raw_frame);

		depth_aligned.convertTo(depth_meter, CV_64FC1);
		depth_meter = depth_meter * depth_scale;

		// Change filename
		sprintf(filename, "%010.3f", timestamp);
		image_filename = image_dir + filename + ".png";
		depth_filename = depth_dir + filename + ".png";
		std::cout << filename << "ms" << std::endl;

		// RGB and depth image
		cv::imshow("color", color);
		cv::imshow("depth_aligned_with_color", depth_aligned);
		cv::imshow("depth_raw", depth_raw);
		cv::imshow("depth_in_meter", depth_meter);
		cv::waitKey(1);

		if(isSave){
			// Save current image and depth
			cv::imwrite(image_filename, color);
			cv::imwrite(depth_filename, depth_aligned);
			rgb_log << filename << " rgb/" << filename << ".png" << std::endl;
			depth_log << filename << " depth/" << filename << ".png" << std::endl;
		}
	}


	return EXIT_SUCCESS;
}


catch(const rs::error & e)
{
	// Method calls against librealsense objects may throw exceptions of type rs::error
	std::cout << "rs::error was thrown when calling " << e.get_failed_function().c_str() << "(" << e.get_failed_args().c_str() << ")" << std::endl;
	std::cout << e.what() << std::endl;
	return EXIT_FAILURE;
}


/* enum class stream : int32_t
   {
   depth                           ,  ///< Native stream of depth data produced by RealSense device
   color                           ,  ///< Native stream of color data captured by RealSense device
   infrared                        ,  ///< Native stream of infrared data captured by RealSense device
   infrared2                       ,  ///< Native stream of infrared data captured from a second viewpoint by RealSense device
   fisheye                         ,
   points                          ,  ///< Synthetic stream containing point cloud data generated by deprojecting the depth image
   rectified_color                 ,  ///< Synthetic stream containing undistorted color data with no extrinsic rotation from the depth stream
   color_aligned_to_depth          ,  ///< Synthetic stream containing color data but sharing intrinsic of depth stream
   infrared2_aligned_to_depth      ,  ///< Synthetic stream containing second viewpoint infrared data but sharing intrinsic of depth stream
   depth_aligned_to_color          ,  ///< Synthetic stream containing depth data but sharing intrinsic of color stream
   depth_aligned_to_rectified_color, ///< Synthetic stream containing depth data but sharing intrinsic of rectified color stream
   depth_aligned_to_infrared2        ///< Synthetic stream containing depth data but sharing intrinsic of second viewpoint infrared stream
   }; */
