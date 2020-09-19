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

#include "stdafx.h"
#include "ImageGrabber.h"
#include <iostream>

using namespace std;

ImageGrabber::ImageGrabber() : 
m_camConfig(), 
m_gpsConfig()
{    
    LadybugError error;
    error = ladybugCreateContext(&m_context);    
    if (error != LADYBUG_OK)
    {
        throw std::runtime_error("Unable to create Ladybug context.");
    }

    error = ladybugCreateGPSContext(&m_gpsContext);
    if (error != LADYBUG_OK)
    {
        throw std::runtime_error("Unable to create GPS context.");
    }
}

ImageGrabber::~ImageGrabber()
{
    ladybugDestroyGPSContext(&m_gpsContext);
    ladybugDestroyContext(&m_context);
}

LadybugError ImageGrabber::Init()
{
    LadybugError error;
    LadybugCameraInfo enumeratedCameras[16];
    unsigned int numCameras = 16;

    error = ladybugBusEnumerateCameras(m_context, enumeratedCameras, &numCameras);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    cout << "Cameras detected: " << numCameras << endl << endl;

    if (numCameras == 0)
    {
        cerr << "Insufficient number of cameras detected." << endl;
        return LADYBUG_FAILED;
    }
    
    error = ladybugInitializeFromIndex(m_context, 0);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    LadybugCameraInfo camInfo;
    error = ladybugGetCameraInfo(m_context, &camInfo);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    cout << "Camera information: " << endl << ladybugCameraInfo::toString(camInfo) << endl;

    return error;
}

LadybugError ImageGrabber::GetCameraInfo( LadybugCameraInfo& camInfo )
{
    const LadybugError error = ladybugGetCameraInfo(m_context, &camInfo);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    return error;
}

void ImageGrabber::SetConfiguration( const CameraConfiguration& camConfig, const GpsConfiguration& gpsConfig )
{
    m_camConfig = camConfig;
    m_gpsConfig = gpsConfig;
}

LadybugError ImageGrabber::Start()
{    
    LadybugError error;
    error = ladybugStartLockNext(m_context, m_camConfig.dataFormat);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    if (m_gpsConfig.useGps)
    {
        error = ladybugRegisterGPS(m_context, &m_gpsContext);
        if (error != LADYBUG_OK)
        {
            return error;
        }

#ifdef _WIN32
        error = ladybugInitializeGPS(m_gpsContext, m_gpsConfig.port, m_gpsConfig.baudRate, m_gpsConfig.refreshInterval);
#else
        error = ladybugInitializeGPSEx(m_gpsContext, m_gpsConfig.deviceName.c_str(), m_gpsConfig.baudRate, m_gpsConfig.refreshInterval);
#endif
        if (error != LADYBUG_OK)
        {
            return error;
        }

        error = ladybugStartGPS(m_gpsContext);
        if (error != LADYBUG_OK)
        {
            return error;
        }
    }    

    error = ladybugSetAbsPropertyEx(m_context, LADYBUG_FRAME_RATE, false, true, m_camConfig.isFrameRateAuto, m_camConfig.frameRate);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    error = ladybugSetJPEGQuality(m_context, m_camConfig.jpegQualityPercentage);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    error = ladybugSetAutoJPEGQualityControlFlag(m_context, m_camConfig.isJpegQualityAuto);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    error = ladybugSetAutoJPEGBufferUsage(m_context, m_camConfig.jpegBufferPercentage);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    // Perform a quick test to make sure images can be successfully acquired
    for (int i=0; i < 10; i++)
    {
        LadybugImage tempImage;
        error = ladybugLockNext(m_context, &tempImage);
    }

    error = ladybugUnlockAll(m_context);
    if (error != LADYBUG_OK)
    {
        return error;
    }

    return error;
}

LadybugError ImageGrabber::Stop()
{
    if (m_gpsConfig.useGps)
    {
        LadybugError gpsError = ladybugStopGPS(m_gpsContext);
        if (gpsError != LADYBUG_OK)
        {
            cerr << "Error: Unable to stop GPS (" << ladybugErrorToString(gpsError) << ")" << endl;
        }

        gpsError = ladybugUnregisterGPS(m_context, &m_gpsContext);
        if (gpsError != LADYBUG_OK)
        {
            cerr << "Error: Unable to unregister GPS (" << ladybugErrorToString(gpsError) << ")" << endl;
        }
    }    

    const LadybugError cameraError = ladybugStop(m_context);
    if (cameraError != LADYBUG_OK)
    {
        cerr << "Error: Unable to stop camera (" << ladybugErrorToString(cameraError) << ")" << endl;
    }

    return cameraError;
}

LadybugError ImageGrabber::Acquire( LadybugImage& image )
{
    return ladybugLockNext(m_context, &image);
}

LadybugError ImageGrabber::Unlock( unsigned int bufferIndex )
{
    return ladybugUnlock(m_context, bufferIndex);
}
