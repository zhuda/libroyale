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
#include <royale/DepthData.hpp>
#include <royale/Vector.hpp>
#include <memory>
#include <cstdint>
#include <chrono>

namespace royale
{
    /**
     *  In addition to the standard depth point, the intermediate point also stores
     *  information which is calculated as temporaries in the processing pipeline.
     */
    struct IntermediatePoint
    {
        float       depth;                //!< depth value of the current sequence
        float       amplitude;            //!< amplitude value of the current sequence
        float       intensity;            //!< intensity value of the current sequence
        uint32_t    flags;                //!< flags of the current sequence
    };

    /**
     *  This structure defines the Intermediate depth data which is delivered through the callback
     *  if the user has access level 2 for the CameraDevice.
     */
    struct IntermediateData
    {
        int                                        version;                    //!< version number of the data format
        std::chrono::milliseconds                  timeStamp;                  //!< timestamp in milliseconds precision (time since epoch 1970)
        uint16_t                                   width;                      //!< width of depth image
        uint16_t                                   height;                     //!< height of depth image
        Vector<Vector<IntermediatePoint>>          points;                     //!< array of intermediate points for each sequence
        Vector<uint32_t>                           modulationFrequencies;      //!< modulation frequencies for each sequence
        Vector<uint32_t>                           exposureTimes;              //!< integration times for each sequence
        uint32_t                                   numFrequencies;             //!< number of processed frequencies
    };
}