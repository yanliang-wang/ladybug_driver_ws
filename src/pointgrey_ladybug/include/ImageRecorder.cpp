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
#include "ImageRecorder.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>

using namespace std;

ImageRecorder::ImageRecorder(const StreamConfiguration& streamConfig) : m_streamConfig(streamConfig)
{
    const LadybugError error = ladybugCreateStreamContext(&m_streamContext);
    if (error != LADYBUG_OK)
    {
        throw std::runtime_error("Unable to create Ladybug stream context");
    }
}

ImageRecorder::~ImageRecorder()
{
    ladybugDestroyStreamContext(&m_streamContext);
}

LadybugError ImageRecorder::Init( LadybugContext cameraContext, unsigned int serialNumber )
{    
    const boost::posix_time::ptime currTime = boost::posix_time::second_clock::local_time();
    const boost::gregorian::date currDate = currTime.date();
    const boost::posix_time::time_duration currTod = currTime.time_of_day();

    char uniqueFilename[128] = {0};
    sprintf(
        uniqueFilename, 
        "ladybug_%u_%04d%02d%02d_%02d%02d%02d.pgr",
        serialNumber, 
        (int)currDate.year(),
        (int)currDate.month(),
        (int)currDate.day(),
        (int)currTod.hours(),
        (int)currTod.minutes(),
        (int)currTod.seconds());

    const std::string baseFileName = m_streamConfig.destinationDirectory + "/" + uniqueFilename;

    char openedFileName[256] = {0};
    const LadybugError error = ladybugInitializeStreamForWriting(m_streamContext, baseFileName.c_str(), cameraContext, openedFileName, true);
    if (error != LADYBUG_OK)
    {        
        return error;
    }

    cout << "Opened stream file: " << openedFileName << endl;

    return error;
}

LadybugError ImageRecorder::Stop()
{
    return ladybugStopStream(m_streamContext);
}

LadybugError ImageRecorder::Write( const LadybugImage& image )
{
    return ladybugWriteImageToStream(m_streamContext, &image, NULL, NULL);
}

LadybugError ImageRecorder::Write( const LadybugImage& image, double& mbWritten, unsigned long& imagesWritten )
{
    return ladybugWriteImageToStream(m_streamContext, &image, &mbWritten, &imagesWritten);
}
