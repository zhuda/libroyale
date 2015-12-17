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
#include <vector>
#include <cstdint>

namespace royale
{
    /*!
     * Each camera can be operated in various operation modes. This list is a complete list of modes which are
     * supported by Royale. However, every camera device only allows a sub-set of this list. The ICameraDevice
     * can be asked about the supported operation modes by calling getOperationModes(). This will return a
     * list of supported modes while the first one will be the default mode.
     */
    enum class OperationMode
    {
        MODE_INVALID            = -1,
        MODE_9_5FPS_2000        = 0,        //!< 8+1, 5  FPS, max exposure time 2000 micro seconds
        MODE_9_10FPS_1000       = 1,        //!< 8+1, 10 FPS, max exposure time 1000 micro seconds
        MODE_9_15FPS_700        = 2,        //!< 8+1, 15 FPS, max exposure time  700 micro seconds
        MODE_9_25FPS_450        = 3,        //!< 8+1, 25 FPS, max exposure time  450 micro seconds
        MODE_5_35FPS_600        = 4,        //!< 4+1, 35 FPS, max exposure time  600 micro seconds
        MODE_5_45FPS_500        = 5,        //!< 4+1, 45 FPS, max exposure time  500 micro seconds
        MODE_5_45FPS_1000       = 6,        //!< 4+1, 45 FPS, max exposure time 1000 micro seconds
        MODE_5_35FPS_1000       = 7,        //!< 4+1, 35 FPS, max exposure time 1000 micro seconds
        MODE_9_5FPS_2400_FPGA   = 8,        //!< 8+1, 5  FPS, max exposure time 2400 micro seconds (FPGA)
        MODE_9_10FPS_1100_FPGA  = 9,        //!< 8+1, 10 FPS, max exposure time 1100 micro seconds (FPGA)
        MODE_9_15FPS_800_FPGA   = 10,       //!< 8+1, 15 FPS, max exposure time  800 micro seconds (FPGA)
        MODE_5_20FPS_1700_FPGA  = 11,       //!< 4+1, 20 FPS, max exposure time 1700 micro seconds (FPGA)
        MODE_5_25FPS_1300_FPGA  = 12,       //!< 4+1, 25 FPS, max exposure time 1300 micro seconds (FPGA)
        MODE_5_30FPS_1100_FPGA  = 13,       //!< 4+1, 30 FPS, max exposure time 1100 micro seconds (FPGA)
        MODE_5_35FPS_900_FPGA   = 14,       //!< 4+1, 35 FPS, max exposure time 900 micro seconds (FPGA)
        MODE_2_5FPS_1000_FPGA   = 15,       //!< 1+1, 5 FPS, max exposure time 1000 micro seconds (FPGA)
        MODE_10_15FPS_1000_FPGA = 16,       //!< 4+4+1+1, 15 FPS, max exposure time 1000 micro seconds (FPGA)
        MODE_PLAYBACK           = 17,       //!< Used for playbacks
        NUM_MODES                          //!< number of modes
    };

    /*!
     * For debugging, printable strings corresponding to the OperationMode enumeration.  The
     * returned value is copy of the operation mode name. If the operation mode is not found
     * an empty string will be returned.
     *
     * These strings will not be localized.
     */
    DllExport String getOperationModeName (OperationMode mode);
}
