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
#include <royale/OperationMode.hpp>
#include <royale/Status.hpp>
#include <royale/Vector.hpp>
#include <royale/Pair.hpp>
#include <cstdint>

namespace royale
{
    //!< Indicates that the capture should be greyscale only
    static const uint32_t GRAYSCALE = 0u;

    /**
     *  The CameraCaptureMode defines the low-level parameters for setting up an operation mode
     *  for the ToF camera. Besides framerate. This functionality is only exposed through the
     *  access LEVEL 3 and not available for LEVEL 1/2.
     *  Since not all sequences are supported by the framework, the setCaptureMode() validates
     *  the specified mode and asks the camera module and processing if the mode is supported.
     *
     *  Example:
     *    - The simple capture mode takes four raw frames and delivers one depth image with 45 fps. This is specified
     *      by `CameraCaptureMode(CaptureOutputType::Depth, 45, { { 30000000, 1000 } });`. This specifies a
     *      modulation frequency of 30 Mhz and an exposure value of 1 ms. The data output will be depth information
     *      which can be retrieved by setting a listener for the ICameraDevice.
     */
    struct CameraCaptureMode
    {
        DllExport CameraCaptureMode();

        /**
        *  Constructor which takes all required inputs to setup a capture mode
        *
        * \param frameRate target framerate for the capture mode [fps]
        * \param exposureLimits time limits as pair of [minExposure, maxExposure]
        * \param sequenceInfo pair of [modulationfrequency, exposuretime] for one sequence.
        * The exposure limits for the modulation sequences should be chosen within the boundaries
        * of the exposure limits. In in case of the gray scale sequence the modulation frequency must be MODFREQ_AUTO.
        * The correct exposure time is the automatically assigned by the image.
        * The frequency must be set in Hz in 100 kHz steps (e.g. 20200000 would mean 20.2 Mhz).
        */
        DllExport CameraCaptureMode (uint16_t frameRate,
                                     Pair<uint32_t, uint32_t> exposureLimits,
                                     Vector<Pair<uint32_t, uint32_t>> sequenceInfo);

        /*!
         *  Generate the capture mode based on the pre-defined operation mode.
         *  Throws an exception if OperationMode is not known.
         */
        DllExport static CameraStatus getCaptureMode (const OperationMode &operationMode,
                CameraCaptureMode &captureMode);

        /**
         *  Defines the target framerate
         */
        uint16_t frameRate;

        /**
         *  Holds a tuple for one sequence entry [modfreq,exptime]. The size of the
         *  vector must fit the selected CaptureSequenceMode
         */
        Vector<Pair<uint32_t, uint32_t>> sequenceInformation;

        /**
        *  Holds the exposure time limits as pair of [minExposure, maxExposure]
        */
        Pair<uint32_t, uint32_t> exposureLimits;
    };
}
