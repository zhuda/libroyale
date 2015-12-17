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

#include <royale/DepthData.hpp>

namespace royale
{
    class DllExport IDepthDataReleaser
    {
    public:
        /**
         * Must be called on all DepthData pointers that are provided by the IDepthDataListener
         * interface.  This returns buffer ownership to the processing module, and allows the
         * memory to be reused.
         *
         * This should not be called when using the Royale API, as CameraDevice releases the
         * depth data when that IDepthDataListener callback returns.
         */
        virtual void releaseDepthData (const DepthData *data) = 0;
    };
}

