#include <chrono>
#include <string>

// g++ -DPIPELINE_LOCAL_TEST -I/mnt/btrfs-data/venvs/ml-tutorials/repos/depthai-core/include -I/mnt/btrfs-data/venvs/ml-tutorials/repos/depthai-shared/include -I/mnt/btrfs-data/venvs/ml-tutorials/repos/depthai-shared/libnop/include -I/mnt/btrfs-data/venvs/ml-tutorials/repos/depthai-shared/json/include -I/mnt/btrfs-data/venvs/ml-tutorials/repos/depthai-android-unity-example/depthai-android-api/depthai-android-api/src/main/include depthai_android_api.cpp -o depth -L. -ldepthai

#ifndef PIPELINE_LOCAL_TEST
#include <android/log.h>

#include <libusb/libusb/libusb.h>
#include <jni.h>
#endif

#include <typeinfo>     // for typeid

#include <iostream>
#include <cstdarg>
#include <sstream> 
#include <fstream>
#include <chrono>

#include "depthai/depthai.hpp"

#include "utils.h"
#include "date.h"

#define LOG_TAG "depthaiAndroid"
#define log(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG, __VA_ARGS__)

#ifndef PIPELINE_LOCAL_TEST
JNIEnv* jni_env = NULL;
JavaVM* JVM;

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved)
{
    // Ref: https://stackoverflow.com/a/49947762/13706271
    JVM = vm;
    if (vm->GetEnv((void**)&jni_env, JNI_VERSION_1_6) != JNI_OK)
        log("ERROR: GetEnv failed");

    return JNI_VERSION_1_6;
}

extern "C"
{
#endif
    using namespace std;

    std::shared_ptr<dai::Device> device;
    std::shared_ptr<dai::DataOutputQueue> qRgb, qDisparity, qDepth;

    std::vector<u_char> rgbImageBuffer, colorDisparityBuffer;

    const int disparityWidth = 640;
    const int disparityHeight = 400;

    // Closer-in minimum depth, disparity range is doubled (from 95 to 190):
    std::atomic<bool> extended_disparity{false};
    auto maxDisparity = extended_disparity ? 190.0f :95.0f;

    // Better accuracy for longer distance, fractional disparity 32-levels:
    std::atomic<bool> subpixel{false};
    // Better handling for occlusions:
    std::atomic<bool> lr_check{false};

    struct video_info
    {
        std::shared_ptr<dai::DataOutputQueue> outQ1;
        std::shared_ptr<dai::DataOutputQueue> outQ2;
        std::shared_ptr<dai::DataOutputQueue> outQ3;

        std::shared_ptr<dai::DataOutputQueue> qRgb;
        std::shared_ptr<dai::DataOutputQueue> qDisparity;
        std::shared_ptr<dai::DataOutputQueue> qDepth;

        std::ofstream videoFile1;
        std::ofstream videoFile2;
        std::ofstream videoFile3;

        std::ofstream logfile;

	std::uint64_t frame_counter = 0;
    };

    video_info v_info;

    void api_stop_device()
    {
        v_info.logfile.close();
    }

    void api_log(const char *format, ...)
    {
	std::stringstream ss;

	char buffer[4096];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, 4095, format, args);

	auto now = std::chrono::system_clock::now();
	ss << date::format("%d/%m/%Y %H:%M:%S -- ", now) << buffer;

	va_end (args);

	#ifndef PIPELINE_LOCAL_TEST
        log("%s", ss.str().c_str());
	#else
	std::cout << std::fixed << std::setprecision(3) << ss.str().c_str() << std::endl;		// no effect on date-time format
	#endif
        v_info.logfile << ss.str() << std::endl;
     
        va_end(args);
    }

    void api_open_logfile(std::string external_storage_path)
    {
        std::string logfile_fname = external_storage_path + "/depthai-android-api.log";
        v_info.logfile.open(logfile_fname, std::ios::app);
        api_log("Opening logfile: %s", logfile_fname.c_str());
        v_info.logfile << logfile_fname + " starting..." << std::endl;
    }

    void api_start_device(int rgbWidth, int rgbHeight, const char* external_storage_path)
    {
	api_open_logfile(std::string(reinterpret_cast<char const*>(external_storage_path)));

	#ifndef PIPELINE_LOCAL_TEST
        // libusb
        auto r = libusb_set_option(nullptr, LIBUSB_OPTION_ANDROID_JNIENV, jni_env);
        api_log("libusb_set_option ANDROID_JAVAVM: %s", libusb_strerror(r));
	#endif

        // Create pipeline
        dai::Pipeline pipeline;

        // Define source and output
        auto camRgb = pipeline.create<dai::node::ColorCamera>();
        auto monoLeft = pipeline.create<dai::node::MonoCamera>();
        auto monoRight = pipeline.create<dai::node::MonoCamera>();
        auto stereo = pipeline.create<dai::node::StereoDepth>();

        auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
        auto xoutDisparity = pipeline.create<dai::node::XLinkOut>();
        auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

        xoutRgb->setStreamName("rgb");
        xoutDisparity->setStreamName("disparity");
        xoutDepth->setStreamName("depth");

        // Properties
        camRgb->setPreviewSize(rgbWidth, rgbHeight);
        camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        camRgb->setInterleaved(true);
        camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

        stereo->initialConfig.setConfidenceThreshold(245);
        // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
        stereo->setLeftRightCheck(lr_check);
        stereo->setExtendedDisparity(extended_disparity);
        stereo->setSubpixel(subpixel);

        // Linking
        camRgb->preview.link(xoutRgb->input);
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);
        stereo->disparity.link(xoutDisparity->input);
        stereo->depth.link(xoutDepth->input);

        // Connect to device and start pipeline
        device = std::make_shared<dai::Device>(pipeline, dai::UsbSpeed::SUPER);

        auto device_info = device->getDeviceInfo();
	#ifndef PIPELINE_LOCAL_TEST
        api_log("%s",device_info.toString().c_str());
	#endif

        // Output queue will be used to get the rgb frames from the output defined above
        qRgb = device->getOutputQueue("rgb", 4, false);
        qDisparity = device->getOutputQueue("disparity", 4, false);
        qDepth = device->getOutputQueue("depth", 4, false);

        // Resize image buffers
        rgbImageBuffer.resize(rgbWidth*rgbHeight*4);
        colorDisparityBuffer.resize(disparityWidth*disparityHeight*4);

        api_log("Device Connected!");
        //v_info.logfile << "Device Connected!" << std::endl;
    }

    unsigned int api_get_rgb_image(unsigned char* unityImageBuffer)
    {
        auto inRgb = qRgb->get<dai::ImgFrame>();
        auto imgData = inRgb->getData();

        // Convert from RGB to RGBA for Unity
        int rgb_index = 0;
        int argb_index = 0;
        for (int y = 0; y < inRgb->getHeight(); y ++)
        {
            for (int x = 0; x < inRgb->getWidth(); x ++)
            {
                rgbImageBuffer[argb_index++] = imgData[rgb_index++]; // red
                rgbImageBuffer[argb_index++] = imgData[rgb_index++]; // green
                rgbImageBuffer[argb_index++] = imgData[rgb_index++]; // blue
                rgbImageBuffer[argb_index++] = 255; // alpha

            }
        }

        // Copy the new image to the Unity image buffer
        std::copy(rgbImageBuffer.begin(), rgbImageBuffer.end(), unityImageBuffer);

        // Return the image number
        return inRgb->getSequenceNum();
    }

    unsigned int api_get_color_disparity_image(unsigned char* unityImageBuffer)
    {
        auto inDisparity = qDisparity->get<dai::ImgFrame>();;
        auto disparityData = inDisparity->getData();
        uint8_t colorPixel[3];

        // Convert Disparity to RGBA format for Unity
        int argb_index = 0;
        for (int i = 0; i < disparityWidth*disparityHeight; i++)
        {
            // Convert the disparity to color
            colorDisparity(colorPixel, disparityData[i], maxDisparity);

            colorDisparityBuffer[argb_index++] = colorPixel[0]; // red
            colorDisparityBuffer[argb_index++] = colorPixel[1]; // green
            colorDisparityBuffer[argb_index++] = colorPixel[2]; // blue
            colorDisparityBuffer[argb_index++] = 255; // alpha
        }

        // Copy the new image to the Unity image buffer
        std::copy(colorDisparityBuffer.begin(), colorDisparityBuffer.end(), unityImageBuffer);

        // Return the image number
        return inDisparity->getSequenceNum();
    }


    int api_start_device_record_video(int rgbWidth, int rgbHeight, const char* external_storage_path)
    {
   	std::string ext_storage_path = std::string(reinterpret_cast<char const*>(external_storage_path));
	api_open_logfile(ext_storage_path);

	#ifndef PIPELINE_LOCAL_TEST
        // libusb
        auto r = libusb_set_option(nullptr, LIBUSB_OPTION_ANDROID_JNIENV, jni_env);
        api_log("libusb_set_option ANDROID_JAVAVM: %s", libusb_strerror(r));
	#endif

        std::string fname_prefix = ext_storage_path + "/depthai-video-";

        // Create pipeline
        dai::Pipeline pipeline;
        api_log("Pipeline started");
    
        // Define sources and outputs
        auto camRgb = pipeline.create<dai::node::ColorCamera>();
        auto monoLeft = pipeline.create<dai::node::MonoCamera>();
        auto monoRight = pipeline.create<dai::node::MonoCamera>();
        auto stereo = pipeline.create<dai::node::StereoDepth>();

        auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
        auto xoutDisparity = pipeline.create<dai::node::XLinkOut>();
        auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

        xoutRgb->setStreamName("rgb");
        xoutDisparity->setStreamName("disparity");
        xoutDepth->setStreamName("depth");

        // Properties
        camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

        // Properties
        camRgb->setPreviewSize(rgbWidth, rgbHeight);
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        camRgb->setInterleaved(true);
        camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

        stereo->initialConfig.setConfidenceThreshold(245);
        // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
        stereo->setLeftRightCheck(lr_check);
        stereo->setExtendedDisparity(extended_disparity);
        stereo->setSubpixel(subpixel);

        // Linking
        camRgb->preview.link(xoutRgb->input);
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);
        stereo->disparity.link(xoutDisparity->input);
        stereo->depth.link(xoutDepth->input);



        auto ve1 = pipeline.create<dai::node::VideoEncoder>();
        auto ve2 = pipeline.create<dai::node::VideoEncoder>();
        auto ve3 = pipeline.create<dai::node::VideoEncoder>();
    
        auto ve1Out = pipeline.create<dai::node::XLinkOut>();
        auto ve2Out = pipeline.create<dai::node::XLinkOut>();
        auto ve3Out = pipeline.create<dai::node::XLinkOut>();

        ve1Out->setStreamName("ve1Out");
        ve2Out->setStreamName("ve2Out");
        ve3Out->setStreamName("ve3Out");
    
        // Create encoders, one for each camera, consuming the frames and encoding them using H.264 / H.265 encoding
        ve1->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::H264_MAIN);      // left
        ve2->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::H265_MAIN);      // RGB
        ve3->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::H264_MAIN);      // right
        api_log("H.264/H.265 video encoders created");
    
        // Linking
        monoLeft->out.link(ve1->input);
        camRgb->video.link(ve2->input);
        monoRight->out.link(ve3->input);

        api_log("H.264/H.265 video encoders linking done");

        ve1->bitstream.link(ve1Out->input);
        ve2->bitstream.link(ve2Out->input);
        ve3->bitstream.link(ve3Out->input);

        api_log("H.264/H.265 video encoders bitstream linking done");

        #if 0
	device = std::make_shared<dai::Device>(pipeline, dai::UsbSpeed::SUPER);
        #endif
	bool exception_thrown = false;
	try
	{
	        // Connect to device and start pipeline
	        device = std::make_shared<dai::Device>(pipeline, dai::UsbSpeed::SUPER);
	}
	catch (const std::overflow_error & err)
	{
		exception_thrown = true;
        	api_log("Overflow error exception: «%s». Exiting...", err.what());
	} // this executes if f() throws std::overflow_error (same type rule)
	catch (const std::runtime_error & err)
	{
		exception_thrown = true;
        	api_log("Runtime error exception: «%s». Exiting...", err.what());
	} // this executes if f() throws std::underflow_error (base class rule)
	catch (const std::exception & err)
	{
		exception_thrown = true;
        	api_log("Unknown exception: «%s». Exiting...", err.what());
	} // this executes if f() throws std::logic_error (base class rule)
	catch (...)
	{
		exception_thrown = true;
        	api_log("Unknown exception. Exiting...");
	} // this executes if f() throws std::string or int or any other unrelated type
	if (exception_thrown)
		return -1;

        api_log("DepthAI device created");
        auto device_info = device->getDeviceInfo();
	#ifndef PIPELINE_LOCAL_TEST
        api_log("Device info: %s", device_info.toString().c_str());
	#endif


    
        // Output queues will be used to get the encoded data from the output defined above
        auto outQ1 = device->getOutputQueue("ve1Out", 30, true);
        auto outQ2 = device->getOutputQueue("ve2Out", 30, true);
        auto outQ3 = device->getOutputQueue("ve3Out", 30, true);

        api_log("Output queues created");

	std::stringstream curr_date_time;
	// auto now = std::chrono::system_clock::now();
	auto now = return_next_full_second();
	// auto now_s = std::chrono::round<std::chrono::seconds>(now);
	curr_date_time << std::fixed << std::setprecision(2) << date::format("%Y%m%d-%H%M%S", now);		// No effect sadly :(
	auto curr_date_time_str = curr_date_time.str();
	auto pos = curr_date_time_str.find("."); 

        // The H.264/H.265 files are raw stream files (not playable yet)
	std::string left_fn  = fname_prefix + curr_date_time_str.substr(0,pos) + std::string("-left.h264" );
	std::string color_fn = fname_prefix + curr_date_time_str.substr(0,pos) + std::string("-color.h265");
	std::string right_fn = fname_prefix + curr_date_time_str.substr(0,pos) + std::string("-right.h264");
        v_info.videoFile1 = std::ofstream(left_fn , std::ios::binary);
        v_info.videoFile2 = std::ofstream(color_fn, std::ios::binary);
        v_info.videoFile3 = std::ofstream(right_fn, std::ios::binary);
        api_log("Output files opened (%s - %s - %s)", left_fn.c_str(), color_fn.c_str(), right_fn.c_str());

        v_info.outQ1 = outQ1;
        v_info.outQ2 = outQ2;
        v_info.outQ3 = outQ3;


        // Output queue will be used to get the rgb frames from the output defined above
        qRgb = device->getOutputQueue("rgb", 4, false);
        qDisparity = device->getOutputQueue("disparity", 4, false);
        qDepth = device->getOutputQueue("depth", 4, false);

        v_info.qRgb = qRgb;
        v_info.qDisparity = qDisparity;
        v_info.qDepth = qDepth;


        // Resize image buffers
        rgbImageBuffer.resize(rgbWidth*rgbHeight*4);
        colorDisparityBuffer.resize(disparityWidth*disparityHeight*4);

        api_log("Device Connected!");

	return 0;
    }
    unsigned long api_write_one_video_frame(std::shared_ptr<dai::DataOutputQueue> outQ, std::ofstream & videoFile)
    {
        auto out = outQ->get<dai::ImgFrame>();
        videoFile.write((char*)out->getData().data(), out->getData().size());

        // std::cout << typeid(out->getData().size()).name() << '\n';

        return out->getData().size();   // compressed size of the H.264/H.265 frame
    }
    unsigned long api_get_video_frames()
    {
        /*
            auto out1 = v_info.outQ1->get<dai::ImgFrame>();
            v_info.videoFile1.write((char*)out1->getData().data(), out1->getData().size());
            auto out2 = v_info.outQ2->get<dai::ImgFrame>();
            v_info.videoFile2.write((char*)out2->getData().data(), out2->getData().size());
            auto out3 = v_info.outQ3->get<dai::ImgFrame>();
            v_info.videoFile3.write((char*)out3->getData().data(), out3->getData().size());

        std::shared_ptr<dai::DataOutputQueue> qRgb;
        std::shared_ptr<dai::DataOutputQueue> qDisparity;
        std::shared_ptr<dai::DataOutputQueue> qDepth;
        */
            /*
            std::cout << "Q1 size: " << api_write_one_video_frame(v_info.outQ1, v_info.videoFile1) << std::endl;
            std::cout << "Q2 size: " << api_write_one_video_frame(v_info.outQ2, v_info.videoFile2) << std::endl;
            std::cout << "Q3 size: " << api_write_one_video_frame(v_info.outQ3, v_info.videoFile3) << std::endl;
            */

            api_write_one_video_frame(v_info.outQ1, v_info.videoFile1);
            api_write_one_video_frame(v_info.outQ2, v_info.videoFile2);
            api_write_one_video_frame(v_info.outQ3, v_info.videoFile3);

	    v_info.frame_counter++;
	    if (v_info.frame_counter % 1000 == 0 or true)
 	           api_log("Read video frame no.: %lu", v_info.frame_counter);

            return v_info.frame_counter;
    }
#ifndef PIPELINE_LOCAL_TEST
}
#endif


#ifdef PIPELINE_LOCAL_TEST
int main()
{
        uint64_t frame_no = 0;
        uint64_t rgbFrameNum = 0;
        uint64_t dispFrameNum = 0;
        int RGBWidth  = 640;
        int RGBHeight = 480;
        std::string external_storage_path = "/tmp/";

        int retval = api_start_device_record_video(RGBWidth, RGBHeight, external_storage_path.c_str());
	if (retval == -1)
		return retval;

        unsigned char* _rgbImgPtr  = new unsigned char[rgbImageBuffer.size()];
        unsigned char* _dispImgPtr = new unsigned char[colorDisparityBuffer.size()];
        std::cout << "rgbImageBuffer.size(): " << rgbImageBuffer.size() << std::endl;
        std::cout << "colorDisparityBuffer.size(): " << colorDisparityBuffer.size() << std::endl;

        while (true)
        {
                frame_no = api_get_video_frames();
                std::cout << "Received frame: " << frame_no << std::endl;
                rgbFrameNum = api_get_rgb_image(_rgbImgPtr);
                dispFrameNum = api_get_color_disparity_image(_dispImgPtr);

        }
}
#endif

