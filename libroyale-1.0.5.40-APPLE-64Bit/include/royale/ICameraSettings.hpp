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

#include <royale/Definitions.hpp>
#include <royale/String.hpp>

namespace royale
{
    /**
     *  Defines an interface for getting the most important camera settings (e.g. calibration path)
     */
    class DllExport ICameraSettings
    {
    public:
        virtual ~ICameraSettings() {}
        /**
         *  Delivers the full path and file name for the calibration file on the file system (if available).
         *  If no calibration file is available, then this method returns false, else the full file name
         *  is returned in fileName. Path delimiter must be '/'
         *
         *  @param fileName contains the full path of the calibration file and will be empty if no
         *         calibration file is available
         *
         *  @return true if calibration file is available, else false
         */
        virtual bool getCalibrationFileName (String &fileName) const = 0;
    };
}
