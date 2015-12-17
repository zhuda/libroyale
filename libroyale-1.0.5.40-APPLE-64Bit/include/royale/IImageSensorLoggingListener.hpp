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
#include <chrono>
#include <royale/String.hpp>

namespace royale
{
    enum class ImageSensorLogType
    {
        Reset,          //!< Logged whenever the image sensor requests a reset (levels are high or low)
        BurstStart,     //!< Indicates an I2C burst start
        BurstEnd,       //!< Indicates that an I2C burst transmission stopped
        I2CWrite,       //!< Logged whenever an I2C write command is initiated by the image sensor (address and value separated by ";")
        I2CRead,        //!< Logged whenever an I2C read command is initiated by the image sensor (address and value separated by ";")
        FPGARegWrite,   //!< Logged whenever an FPGA write register command is initiated by the image sensor (address and value separated by ";")
        FPGARegRead,    //!< Logged whenever an FPGA read register command is initiated by the image sensor (address and value separated by ";")
        Comment,        //!< For general information about the image senor operation (just a message string)
        Sleep,          //!< Logged whenever an active wait state is initiated (time in units of us)
        NUM_LOGTYPES
    };

    /**
    * This interface allows observers to register for logging information from
    * the image sensor interface.
    */
    class DllExport IImageSensorLoggingListener
    {
    public:
        virtual ~IImageSensorLoggingListener() {}

        /**
        * Will be called by the image sensor implementation on any operation
        * that has impact to the image sensor hardware.
        *
        * \param serial     The serial number of the sender
        * \param logTime    Start time of the logged action
        * \param logType    Generic type of the logged action
        * \param logMessage Detailed description of what happens
        */
        virtual void onImagerLogEvent (const String &serial, std::chrono::high_resolution_clock::time_point logTime, ImageSensorLogType logType, const String &logMessage) = 0;
    };
}
