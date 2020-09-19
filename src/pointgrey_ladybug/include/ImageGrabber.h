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

#ifndef ImageGrabber_h__
#define ImageGrabber_h__

#include "Configuration.h"

class ImageGrabber
{
public:
    ImageGrabber();
    ~ImageGrabber();

    LadybugError Init();

    LadybugError GetCameraInfo(LadybugCameraInfo& camInfo);

    void SetConfiguration(const CameraConfiguration& camConfig, const GpsConfiguration& gpsConfig);

    LadybugError Start();
    LadybugError Stop();

    LadybugError Acquire(LadybugImage& image);
    LadybugError Unlock(unsigned int bufferIndex);

    LadybugContext GetCameraContext() const { return m_context; }

private:
    LadybugContext m_context;
    LadybugGPSContext m_gpsContext;

    CameraConfiguration m_camConfig;
    GpsConfiguration m_gpsConfig;
};

#endif // ImageGrabber_h__
