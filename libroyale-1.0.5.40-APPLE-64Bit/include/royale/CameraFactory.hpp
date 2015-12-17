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

#include <memory>

#include <royale/String.hpp>
#include <royale/Definitions.hpp>
#include <royale/ICameraDevice.hpp>
#include <royale/UsbCameraProperties.hpp>

namespace royale
{
    /**
     * LEVEL 3
     * The CameraFactory can be used to create custom camera modules which are not officially supported
     * by Royale. The factory is typically used for bringing up new camera modules.
     */
    class CameraFactory
    {
    public:
        DllExport CameraFactory (const String &activationCode = "");
        DllExport ~CameraFactory();

        /**
         * LEVEL 3
         * Creates a custom camera module by providing a specific UsbCameraProperties object
         *
         * \return nullptr if module cannot be created
         */
        DllExport std::unique_ptr<ICameraDevice> createCamera (const UsbCameraProperties &properties);

        /**
         * LEVEL 3
         * The image sensor logging listener allows to capture the activities of the used Imager component.
         */
        DllExport void addSensorLogListener (IImageSensorLoggingListener *listener);

    private:
        struct CameraFactoryData;
        std::unique_ptr<CameraFactoryData> m_data;
    };
}