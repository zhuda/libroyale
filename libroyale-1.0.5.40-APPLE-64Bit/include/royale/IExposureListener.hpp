/****************************************************************************
 **                                                                        **
 **  Copyright (C) 2015 Infineon Technologies                              **
 **  All rights reserved.                                                  **
 **                                                                        **
 ****************************************************************************/

#pragma once

#include <royale/Vector.hpp>
#include <royale/Definitions.hpp>
#include <cstdint>

namespace royale
{
    /*!
     * Provides the listener interface for handling auto-exposure updates in royale. A listener needs
     * to implement this interface and register itself as a listener to the ICameraDevice.
     */
    class DllExport IExposureListener
    {
    public:
        virtual ~IExposureListener() {}

        /*!
         * Will be called when the newly calculated exposure time deviates from currently set exposure time of the current UseCase.
         *
         * @param exposureTimes Newly calculated exposure time
         */
        virtual void onNewExposure (const Vector<uint32_t> &exposureTimes) = 0;
    };
}
