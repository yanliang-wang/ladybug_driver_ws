//=============================================================================
// Copyright (c) 2001-2018 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id$
//=============================================================================

#ifndef Configuration_h__
#define Configuration_h__

#include "ladybug.h"
#include <cstring>
#include <sstream>
#include <cassert>

using namespace std;

struct GeneralConfiguration
{
    // TODO: General application configuration

    GeneralConfiguration()
    {
        // TODO: Initialize as needed
    }
};

struct CameraConfiguration
{
    LadybugDataFormat dataFormat;
    float frameRate;
    bool isFrameRateAuto;
    unsigned int jpegQualityPercentage;
	bool isJpegQualityAuto;
	unsigned int jpegBufferPercentage; // Range from 0 - 127

    // TODO: Trigger configuration?

    CameraConfiguration()
    {
        dataFormat = LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8;
        frameRate = 16.0f;
        isFrameRateAuto = true;
        jpegQualityPercentage = 80;
	    isJpegQualityAuto = true;
	    jpegBufferPercentage = 110;
    }

    /** Better initialization for the type of camera. */
    CameraConfiguration(LadybugDeviceType deviceType)
    {
        switch (deviceType)
        {
        case LADYBUG_DEVICE_COMPRESSOR:
            {
                dataFormat = LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8;
                frameRate = 16.0f;
                isFrameRateAuto = true;
                jpegQualityPercentage = 80;
                isJpegQualityAuto = true;
                jpegBufferPercentage = 110;
            }
            break;

        case LADYBUG_DEVICE_LADYBUG3:
            {
                dataFormat = LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8;
                frameRate = 16.0f;
                isFrameRateAuto = true;
                jpegQualityPercentage = 80;
                isJpegQualityAuto = true;
                jpegBufferPercentage = 110;
            }
            break;

        case LADYBUG_DEVICE_LADYBUG5:
            {
                dataFormat = LADYBUG_DATAFORMAT_COLOR_SEP_JPEG12;
                frameRate = 10.0f;
                isFrameRateAuto = true;
                jpegQualityPercentage = 80;
                isJpegQualityAuto = true;
                jpegBufferPercentage = 110;
            }
            break;
        case LADYBUG_DEVICE_LADYBUG5P:
        {
            dataFormat = LADYBUG_DATAFORMAT_COLOR_SEP_JPEG12;
            frameRate = 30.0f;
            isFrameRateAuto = true;
            jpegQualityPercentage = 80;
            isJpegQualityAuto = true;
            jpegBufferPercentage = 110;
        }
        break;

        default: assert(false); break;
        }
    }

    std::string ToString()
    {
        std::stringstream output;
        output << " Camera Configuration" << endl;
        output << " Data format: " << dataFormat::toPrettyString(dataFormat) << endl;
        output << " Frame rate: " << frameRate << endl;
        output << " Use auto frame rate: " << (isFrameRateAuto ? "Yes" : "No") << endl;
        output << " JPEG quality: " << jpegQualityPercentage << "%" << endl;
        output << " Use auto JPEG quality: " << (isJpegQualityAuto ? "Yes" : "No") << endl;
        output << " JPEG buffer percentage: " << jpegBufferPercentage << endl;

        return output.str();
    }
};

struct GpsConfiguration
{
    bool useGps;
    unsigned int port;
    std::string deviceName;
    unsigned int baudRate;
    unsigned int refreshInterval;

    GpsConfiguration()
    {
        useGps = false;
        port = 1;
        deviceName = "/dev/ttyACM0";
        baudRate = 115200;
        refreshInterval = 1000;
    }

    std::string ToString()
    {
        std::stringstream output;
        output << "GPS Configuration" << endl;
        output << " Use GPS: " << (useGps ? "Yes" : "No") << endl;
        output << " Port: " << port << endl;
        output << " Device name: " << deviceName << endl;        
        output << " Baud rate: " << baudRate << endl;
        output << " Refresh interval (ms): " << refreshInterval << endl;

        return output.str();
    }
};

struct StreamConfiguration
{
    std::string destinationDirectory;

    StreamConfiguration()
    {
        destinationDirectory = ".";
    }

    std::string ToString()
    {
        std::stringstream output;
        output << "Stream Configuration" << endl;
        output << " Destination directory: " << destinationDirectory << endl;

        return output.str();
    }
};

struct ConfigurationProperties 
{
    GeneralConfiguration general;
    CameraConfiguration camera;
    GpsConfiguration gps;
    StreamConfiguration stream;    

    ConfigurationProperties()
    {
        general = GeneralConfiguration();
        camera = CameraConfiguration(); 
        gps = GpsConfiguration();
        stream = StreamConfiguration();
    }

    ConfigurationProperties(LadybugDeviceType deviceType)
    {
        general = GeneralConfiguration();
        camera = CameraConfiguration(deviceType); 
        gps = GpsConfiguration();
        stream = StreamConfiguration();
    }

    std::string ToString()
    {
        std::stringstream output;
        output << "*** Configuration ***" << endl;
        output << camera.ToString() << gps.ToString() << stream.ToString() << endl;

        return output.str();
    }
};

#endif // Configuration_h__