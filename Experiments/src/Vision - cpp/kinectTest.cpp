#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

using namespace std;
using namespace cv;

enum
{
	Processor_cl,
	Processor_gl,
	Processor_cpu
};

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
	protonect_shutdown = true;
}

int main()
{
	std::cout << "Hello World!" << std::endl;

	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = NULL;
	libfreenect2::PacketPipeline  *pipeline = NULL;

	if(freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
		return -1;
	}

	string serial = freenect2.getDefaultDeviceSerialNumber();

	std::cout << "SERIAL: " << serial << std::endl;

#if 1 // sean
	int depthProcessor = Processor_cl;

	if(depthProcessor == Processor_cpu)
	{
		if(!pipeline)
			//! [pipeline]
			pipeline = new libfreenect2::CpuPacketPipeline();
		//! [pipeline]
	}
	else if (depthProcessor == Processor_gl) // if support gl
	{
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
		if(!pipeline)
		{
			pipeline = new libfreenect2::OpenGLPacketPipeline();
		}
#else
		std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
	}
	else if (depthProcessor == Processor_cl) // if support cl
	{
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		if(!pipeline)
			pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
		std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
	}

	if(pipeline)
	{
		dev = freenect2.openDevice(serial, pipeline);
	}
	else
	{
		dev = freenect2.openDevice(serial);
	}

	if(dev == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;

	libfreenect2::SyncMultiFrameListener listener(
			libfreenect2::Frame::Color |
			libfreenect2::Frame::Depth |
			libfreenect2::Frame::Ir);
	libfreenect2::FrameMap frames;

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);

	dev->start();

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
	// sean: that is a driver bug
	// check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger

	Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

	cv::namedWindow("rgb", WND_PROP_ASPECT_RATIO);
	cv::namedWindow("ir", WND_PROP_ASPECT_RATIO);
	cv::namedWindow("depth", WND_PROP_ASPECT_RATIO);
	cv::namedWindow("undistorted", WND_PROP_ASPECT_RATIO);
	cv::namedWindow("registered", WND_PROP_ASPECT_RATIO);
	cv::namedWindow("depth2RGB", WND_PROP_ASPECT_RATIO);

	while(!protonect_shutdown)
	{
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

		cv::imshow("rgb", rgbmat);
		cv::imshow("ir", irmat / 4500.0f);
		cv::imshow("depth", depthmat / 4500.0f);

		registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

		cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
		cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
		cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

		cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
		cv::imshow("registered", rgbd);
		cv::imshow("depth2RGB", rgbd2 / 4500.0f);

		int key = cv::waitKey(1);
		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

		listener.release(frames);
	}

	dev->stop();
	dev->close();

	delete registration;

#endif

	std::cout << "Goodbye World!" << std::endl;
	return 0;
}