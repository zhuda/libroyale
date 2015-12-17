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

#include <cstdint>
#include <string>
#include <royale/Vector.hpp>
#include <royale/Pair.hpp>
#include <royale/String.hpp>

namespace royale
{
    /**
    * Defines the interface used for the imager. This may be different to the external interface from the module,
    * for example a module may also include conversion hardware from this format to HDMI or USB.
    */
    enum class DataInterface
    {
        PIF,
        CSI2
    };

    /**
     * Defines the lens center which should be used for centering the imager ROI
     */
    struct LensCenterDesign
    {
        uint16_t column;
        uint16_t row;
    };

    /**
     * Defines the type of imager which is used for the camera module
     */
    enum class ImagerType
    {
        IRS1020C_A11,
        IRS1020C_A12,
        IRS1645C,
        NUM_IMAGERTYPES
    };

    /**
     * Defines the type of temperature sensor which is used for the camera module
     */
    enum class TemperatureSensor
    {
        MCP98x43,
        TMP102,
        NUM_TEMPERATURESENSORS
    };

    /**
     * The UsbCameraProperties can be used for creating custom camera modules. Customer
     * modules can only be created in access level L3 by using the CameraFactory. The
     * UsbCameraProperties currently only supports USB-based modules and requires a correct
     * VID/PID.
    */
    struct UsbCameraProperties
    {
        String         description;            //!< Informal description of the camera properties

        uint16_t       maxImageWidth;          //!< Denotes the module's maximal width

        uint16_t       maxImageHeight;         //!< Denotes the module's maximal height

        ImagerType     imagerType;             //!< Identifies the imager silicon name and design step

        uint32_t       systemFrequency;        //!< The XTAL frequency

        uint32_t       maxModulationFrequency; //!< Denotes the module's maximal modulation frequency

        float          interfaceDelay;         //!< Defines the time the readout of a row/line of the sensor area should be delayed

        //!< Base configuration for the imager
        Vector< Pair<uint16_t, uint16_t> >    baseConfig;

        //!< User configuration for the imager
        Vector< Pair<uint16_t, uint16_t> >    userConfig;

        LensCenterDesign                      lensCenterDesign;
        DataInterface                         dataInterface;

        //!< Denotes the vendor ID for the used USB chip
        uint16_t                              vendorId;

        //!< Denotes the product ID for the used USB chip
        uint16_t                              productId;

        TemperatureSensor                     temperatureSensor;

        //!< Denotes the I2C address which is used talking to the temperature sensor
        uint8_t                               temperatureSensorAddress;
    };
}
