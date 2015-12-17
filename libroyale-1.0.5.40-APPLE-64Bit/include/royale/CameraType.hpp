/****************************************************************************\
 * Copyright (C) 2015 Infineon Technologies & pmdtechnologies gmbh
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#pragma once

namespace royale
{
    /*!
     * The CameraType identifies the connected camera device. Only listed types are
     * supported by the Royale API.
     */
    enum class CameraType
    {
        PICOS_STANDARD          = 1,          //!< standard PicoS camera module
        PICOFLEXX               = 2,          //!< standard PicoFlexx camera module

        EVALBOARD_LED_STANDARD  = 10,         //!< standard EvalBoard camera module

        PMD_PLATFORM            = 100,        //!< standard FPGA camera module

        CUSTOM                  = 10000,      //!< custom module

        UNKNOWN                 = 0x7fffff01 //!< indicates that the camera device is not recognized
    };
}
