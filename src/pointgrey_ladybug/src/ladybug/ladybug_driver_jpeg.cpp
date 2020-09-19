//=============================================================================
//
// ladybug_driver_jpeg.cpp
// 2020.9.19
//
//
// This program is used to grab jpeg format images and save into a Ladybug source stream(.pgr)
// The name of output stream file is "ladybug_12193139_20200919_144627-000000.pgr"
// ATTENTION: The name of the stream must not be changed!!!
//
//=============================================================================
//
// example:
//      roslaunch pointgrey_ladybug save_jpeg_data.launch
//
//=============================================================================


#include <iostream>
#include <string>
#include "ladybug.h"

#include "stdafx.h"
#include "Configuration.h"
#include "ConfigurationLoader.h"
#include "ImageGrabber.h"
#include "ImageRecorder.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <chrono>


using namespace std;

static volatile int running_ = 1;

ros::Publisher pub_imageId;

//LadybugDataFormat m_dataFormat;
// camera config settings
float m_frameRate ;
int m_jpegQualityPercentage , m_jpegBufferPercentage;
bool m_isFrameRateAuto , m_isJpegQualityAuto , m_useGps;
string m_dataFormat , m_destinationDirectory;
/**
 * Main method, that will startup the camera
 * This will also make all the ROS publishers needed
 */
int main (int argc, char **argv)
{
    ////ROS STUFF
    ros::init(argc, argv, "ladybug_camera");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    ConfigurationProperties config;
    {
        // Read in our launch parameters
        private_nh.param<string>("DataFormat", m_dataFormat, "LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8");
        private_nh.param<float>("FrameRate", m_frameRate, 16);
        private_nh.param<bool>("IsFrameRateAuto", m_isFrameRateAuto, true);
        private_nh.param<int>("JpegQualityPercentage", m_jpegQualityPercentage, 80);
        private_nh.param<bool>("IsJpegQualityAuto", m_isJpegQualityAuto, true);
        private_nh.param<int>("JpegBufferPercentage", m_jpegBufferPercentage, 110);
        private_nh.param<bool>("UseGps", m_useGps, false);
        private_nh.param<string>("DestinationDirectory", m_destinationDirectory, ".");

        // Camera
        config.camera.dataFormat = dataFormat::fromString(m_dataFormat);
        config.camera.frameRate = m_frameRate;
        config.camera.isFrameRateAuto = m_isFrameRateAuto;
        config.camera.jpegQualityPercentage = m_jpegQualityPercentage;
        config.camera.isJpegQualityAuto = m_isJpegQualityAuto;
        config.camera.jpegBufferPercentage = m_jpegBufferPercentage;

        // GPS
        config.gps.useGps = m_useGps;

        // Stream
        config.stream.destinationDirectory = m_destinationDirectory;
    }
    cout << config.ToString() << endl;

    // Initialize grabber
    ImageGrabber grabber;
    const LadybugError grabberInitError = grabber.Init();
    if (grabberInitError != LADYBUG_OK)
    {
        cerr << "Error: " << "Failed to initialize camera (" << ladybugErrorToString(grabberInitError) << ")" << endl;
        return -1;
    }
    grabber.SetConfiguration(config.camera, config.gps);

    // Get the camera information
    LadybugCameraInfo camInfo;
    grabber.GetCameraInfo(camInfo);

    // Initialize recorder
    ImageRecorder recorder(config.stream);
    const LadybugError recorderInitError = recorder.Init(grabber.GetCameraContext(), camInfo.serialBase);
    if (recorderInitError != LADYBUG_OK)
    {
        std::string additionalInformation = "";

        if (recorderInitError == LADYBUG_COULD_NOT_OPEN_FILE)
        {
            additionalInformation = " This may be caused by permission issues with the destination directory. Try setting the desination directory to a location that does not require admin privilege.";
        }

        cerr << "Error: " << "Failed to initialize stream (" << ladybugErrorToString(recorderInitError) << ")." << additionalInformation << endl;
        return -1;
    }

    const LadybugError startError = grabber.Start();
    if (startError != LADYBUG_OK)
    {
        cerr << "Error: " << "Failed to start camera (" << ladybugErrorToString(startError) << ")" << endl;
        return -1;
    }

    // Create the publishers
    ROS_INFO("Successfully started ladybug camera and stream");
    pub_imageId = n.advertise<std_msgs::Header>( "/ladybug/imageId",100);
    std_msgs::Header imageHeader;
    // Start camera polling loop , grab and record to the stream file
    LadybugImage currentImage;
//    chrono::steady_clock::time_point t1 ;
//    chrono::steady_clock::time_point t2 ;
//    chrono::duration<double> time_used;
    while (running_ && ros::ok())
    {
        //grab
//        t1 = chrono::steady_clock::now();
        const LadybugError acquisitionError = grabber.Acquire(currentImage);
        if (acquisitionError != LADYBUG_OK)
        {
            // Error
            cerr << "Failed to acquire image. Error (" << ladybugErrorToString(acquisitionError) << ")" << endl;
            continue;
        }

        imageHeader.seq = currentImage.imageInfo.ulSequenceId;
        imageHeader.frame_id ="/ladybug";
        imageHeader.stamp = ros::Time::now();
        pub_imageId.publish(imageHeader); // publish image information
        cout << currentImage.imageInfo.ulTimeSeconds <<" "<<currentImage.imageInfo.ulTimeMicroSeconds<< endl;
        cout << imageHeader.stamp.sec <<" "<<  uint32_t (imageHeader.stamp.nsec/1000)<< endl;

//        t2 = chrono::steady_clock::now();
//        time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
//        cout << "Image acquired - " << currentImage.timeStamp.ulCycleSeconds << ":" << currentImage.timeStamp.ulCycleCount << endl;

        // record
        double mbWritten = 0.0;
        unsigned long imagesWritten = 0;
        const LadybugError writeError = recorder.Write(currentImage, mbWritten, imagesWritten);
        if (writeError != LADYBUG_OK)
        {
            // Error
            cerr << "Failed to write image to stream (" << ladybugErrorToString(writeError) << ")" << endl;
            continue;
        }

        cout << imagesWritten << " images - " << mbWritten << "MB" << endl;

        grabber.Unlock(currentImage.uiBufferIndex);

    }

    // Shutdown, and disconnect camera
    ROS_INFO("Stopping ladybug_camera...");
    grabber.Stop();

    // Done!
    ROS_INFO("ladybug_camera stopped");
    return EXIT_SUCCESS;
}
