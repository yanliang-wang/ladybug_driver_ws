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

#include "stdafx.h"
#include "LadybugFunctors.h"
#include <sstream>

LadybugDataFormat dataFormat::fromString(std::string format)
{
    if (format == "LADYBUG_DATAFORMAT_RAW8") { return LADYBUG_DATAFORMAT_RAW8; }
    if (format == "LADYBUG_DATAFORMAT_JPEG8") { return LADYBUG_DATAFORMAT_JPEG8; }
    if (format == "LADYBUG_DATAFORMAT_COLOR_SEP_RAW8") { return LADYBUG_DATAFORMAT_COLOR_SEP_RAW8; }
    if (format == "LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8") { return LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8; }
    if (format == "LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW8") { return LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW8; }
    if (format == "LADYBUG_DATAFORMAT_COLOR_SEP_HALF_HEIGHT_JPEG8") { return LADYBUG_DATAFORMAT_COLOR_SEP_HALF_HEIGHT_JPEG8; }
    if (format == "LADYBUG_DATAFORMAT_RAW16") { return LADYBUG_DATAFORMAT_RAW16; }
    if (format == "LADYBUG_DATAFORMAT_COLOR_SEP_JPEG12") { return LADYBUG_DATAFORMAT_COLOR_SEP_JPEG12; }
    if (format == "LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW16") { return LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW16; }
    if (format == "LADYBUG_DATAFORMAT_COLOR_SEP_HALF_HEIGHT_JPEG12") { return LADYBUG_DATAFORMAT_COLOR_SEP_HALF_HEIGHT_JPEG12; }
    if (format == "LADYBUG_DATAFORMAT_RAW12") { return LADYBUG_DATAFORMAT_RAW12; }
    if (format == "LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW12") { return LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW12; }
    if (format == "LADYBUG_DATAFORMAT_ANY") { return LADYBUG_DATAFORMAT_ANY; }

    return LADYBUG_DATAFORMAT_ANY;
}

std::string dataFormat::toPrettyString( LadybugDataFormat format )
{
    switch (format)
    {
    case LADYBUG_DATAFORMAT_RAW8: return std::string("Uncompressed (8-bit)");
    case LADYBUG_DATAFORMAT_COLOR_SEP_JPEG8: return std::string("JPEG (8-bit)");
    case LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW8: return std::string("Uncompressed Half Height (8-bit)");
    case LADYBUG_DATAFORMAT_COLOR_SEP_HALF_HEIGHT_JPEG8: return std::string("JPEG Half Height (8-bit)");
    case LADYBUG_DATAFORMAT_RAW16: return std::string("Uncompressed (16-bit)");
    case LADYBUG_DATAFORMAT_COLOR_SEP_JPEG12: return std::string("JPEG (12-bit)");
    case LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW16: return std::string("Uncompressed Half Height (16-bit)");
    case LADYBUG_DATAFORMAT_COLOR_SEP_HALF_HEIGHT_JPEG12: return std::string("JPEG Half Height (12-bit)");
    case LADYBUG_DATAFORMAT_RAW12: return std::string("Uncompressed (12-bit)");
    case LADYBUG_DATAFORMAT_HALF_HEIGHT_RAW12: return std::string("Uncompressed Half Height (12-bit)");
    default: return std::string("Unknown");
    }
}

std::string ladybugCameraInfo::toString( const LadybugCameraInfo& cameraInfo )
{
    std::stringstream output;

    output << "Base s/n: " << cameraInfo.serialBase << std::endl;
    output << "Head s/n: " << cameraInfo.serialHead << std::endl;
    output << "Model: " << cameraInfo.pszModelName << std::endl;
    output << "Sensor: " << cameraInfo.pszSensorInfo << std::endl;
    output << "Vendor: " << cameraInfo.pszVendorName << std::endl;
    output << "Bus / Node: " << cameraInfo.iBusNum << " / " << cameraInfo.iNodeNum << std::endl;

    return output.str();
}
