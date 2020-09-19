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
#include "ConfigurationLoader.h"
#include "LadybugRecorderConsoleConfiguration.h"
#include <boost/filesystem.hpp>

namespace
{
    std::shared_ptr<LRCConfig::Configuration> LoadFromFile(std::string fileToLoad)
    {
        // Open and read configuration
        LRCConfig::Configuration* rawConfig = LRCConfig::parseConfiguration(fileToLoad, xml_schema::Flags::dont_validate).release();
        return std::shared_ptr<LRCConfig::Configuration>(rawConfig);        
    }
}

ConfigurationProperties ConfigurationLoader::Parse( std::string fileToLoad )
{   
    // Check if file exists
    if (!boost::filesystem::exists(fileToLoad))
    {
        throw std::runtime_error("Configuration file LadybugRecorderConsole.xml not found in current directory.");
    }
    
    std::shared_ptr<LRCConfig::Configuration> pRawConfig;

    try
    {
        pRawConfig = LoadFromFile(fileToLoad);    
    }
    catch (const xml_schema::Exception& /*e*/)
    {
        throw std::runtime_error("Failed to parse configuration file LadybugRecorderConsole.xml.");
    }    

    // Transfer into output properties    
    ConfigurationProperties outputProps;

    // General

    // Camera
    outputProps.camera.dataFormat = dataFormat::fromString(std::string(pRawConfig->getCamera().getDataFormat().c_str()));
    outputProps.camera.frameRate = pRawConfig->getCamera().getFrameRate();
    outputProps.camera.isFrameRateAuto = pRawConfig->getCamera().getIsFrameRateAuto();
    outputProps.camera.jpegQualityPercentage = pRawConfig->getCamera().getJpegQualityPercentage();
    outputProps.camera.isJpegQualityAuto = pRawConfig->getCamera().getIsJpegQualityAuto();
    outputProps.camera.jpegBufferPercentage = pRawConfig->getCamera().getJpegBufferPercentage();

    // GPS
    outputProps.gps.useGps = pRawConfig->getGPS().getUseGps();
    outputProps.gps.port = pRawConfig->getGPS().getPort();
    outputProps.gps.deviceName = std::string(pRawConfig->getGPS().getDeviceName().c_str());
    outputProps.gps.baudRate = pRawConfig->getGPS().getBaudRate();
    outputProps.gps.refreshInterval = pRawConfig->getGPS().getRefreshIntervalMs();

    // Stream
    outputProps.stream.destinationDirectory = std::string(pRawConfig->getStream().getDestinationDirectory().c_str());
    
    return outputProps;
}
