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

namespace royale
{
    enum class CameraStatus
    {
        SUCCESS                           = 0,    //!< Indicates that there isn't an error

        RUNTIME_ERROR                     = 1024, //!< Something unexpected happened
        DISCONNECTED                      = 1026, //!< Camera device is disconnected
        INVALID_VALUE                     = 1027, //!< The value provided is invalid
        TIMEOUT                           = 1028, //!< The connection got a timeout

        LOGIC_ERROR                       = 2048, //!< This does not make any sense here
        UNKNOWN_HANDLE                    = 2049, //!< Low-level handle is not initialized
        NOT_IMPLEMENTED                   = 2050, //!< This feature is not implemented yet
        OUT_OF_BOUNDS                     = 2051, //!< Setting/parameter is out of specified range

        RESOURCE_ERROR                    = 4096, //!< Cannot access resource
        FILE_NOT_FOUND                    = 4097, //!< Specified file was not found
        COULD_NOT_OPEN                    = 4098, //!< Cannot open file
        DATA_NOT_FOUND                    = 4099, //!< No data available where expected
        END_OF_DATA                       = 4100, //!< We arrived at the end of the data block
        DEVICE_IS_BUSY                    = 4101, //!< I am busy, please do not disturb

        MODULE_NOT_SUPPORTED              = 5000, //!< This camera module is not supported
        OPERATION_MODE_NOT_SUPPORTED      = 5001, //!< This operation mode is not supported
        FRAMERATE_NOT_SUPPORTED           = 5002, //!< The specified frame rate is not supported
        EXPOSURE_TIME_NOT_SUPPORTED       = 5003, //!< The exposure time is not supported
        DEVICE_NOT_INITIALIZED            = 5004, //!< The device seems to be uninitialized
        CALIBRATION_DATA_ERROR            = 5005, //!< The calibration data is not readable
        INSUFFICIENT_PRIVILEGES           = 5006, //!< The camera access level does not allow to call this operation
        DEVICE_ALREADY_INITIALIZED        = 5007, //!< The camera was already initialized
        EXPOSURE_MODE_INVALID             = 5008, //!< The current set exposure mode does not support this operation

        CAPTUREMODE_NOT_SET               = 5100, //!< No capture mode is set
        CAPTUREMODE_ERROR_FRAMERATE       = 5101, //!< Frame rate is out of spec
        CAPTUREMODE_ERROR_PHASE           = 5102, //!< Phase setting is out of spec
        CAPTUREMODE_ERROR_MOD_FREQ        = 5103, //!< Modulation frequency is out of spec
        CAPTUREMODE_ERROR_EXPOSURE_TIME   = 5104, //!< Exposure time is out of spec
        CAPTUREMODE_ERROR_REGION          = 5105, //!< Region settings is out of spec

        FSM_INVALID_TRANSITION            = 8096, //!< Camera module state machine does not support current transition

        UNKNOWN                           = 0x7fffff01 //!< Catch-all failure
    };
}
