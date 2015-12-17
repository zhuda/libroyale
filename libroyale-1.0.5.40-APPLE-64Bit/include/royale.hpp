/****************************************************************************\
 * Copyright (C) 2015 Infineon Technologies
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#pragma once

// This is the top level royale include file

#include <royale/CameraManager.hpp>
#include <royale/ICameraDevice.hpp>

namespace royale
{
    /*!
     * Get the library version of Royale
     */
    DllExport void getVersion (unsigned &major, unsigned &minor, unsigned &patch);
    DllExport void getVersion (unsigned &major, unsigned &minor, unsigned &patch, unsigned &build);
}
