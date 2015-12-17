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
#include <royale/Vector.hpp>
#include <memory>
#include <cstdint>
#include <chrono>

namespace royale
{
    /**
     *  This structure defines the raw data which is delivered through the callback only
     *  exposed for access LEVEL 2.
     *  This data comprises the raw phase images coming directly from the imager.
     */
    struct RawData
    {
        DllExport RawData();
        DllExport RawData (size_t rawVectorSize);

        std::chrono::milliseconds     timeStamp;                    //!< timestamp in milliseconds precision (time since epoch 1970)
        uint16_t                      width;                        //!< width of raw frame
        uint16_t                      height;                       //!< height of raw frame
        Vector<const uint16_t *>      rawData;                      //!< pointer to each raw frame
        Vector<uint32_t>              modulationFrequencies;        //!< modulation frequencies for each sequence
        Vector<uint32_t>              exposureTimes;                //!< integration times for each sequence
        float                         illuminationTemperature;      //!< temperature of illumination
    };

}
