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

#include <royale/Status.hpp>
#include <royale/ICameraSettings.hpp>
#include <royale/IDepthDataListener.hpp>
#include <royale/IExtendedDataListener.hpp>
#include <royale/CameraCaptureMode.hpp>
#include <royale/LensParameters.hpp>
#include <royale/OperationMode.hpp>
#include <royale/CameraType.hpp>
#include <royale/ProcessingFlag.hpp>
#include <royale/IImageSensorLoggingListener.hpp>
#include <royale/IRecordStopListener.hpp>
#include <royale/IExposureListener.hpp>
#include <royale/ExposureMode.hpp>
#include <royale/Vector.hpp>
#include <royale/String.hpp>

namespace royale
{
    /**
     * Specifies the type of data which should be captured and returned as callback.
     */
    enum class CallbackData  : uint16_t
    {
        None = 0x00,          //!< only get the callback but no data delivery
        Raw  = 0x01,          //!< raw frames, if exclusively used no processing pipe is executed (no calibration data is needed)
        Depth = 0x02,         //!< one depth and grayscale image will be delivered for the complete sequence
        Intermediate = 0x04   //!< all intermediate data will be delivered which are generated in the processing pipeline
    };

    enum class CameraAccessLevel
    {
        L1 = 0, //!< Level 1 access provides the depth data using standard, known-working configurations
        L2 = 1, //!< Level 2 access provides the raw data, for example for developing custom processing pipelines
        L3 = 2  //!< Level 3 access is for bringing up new camera modules
    };

    /**
     * This is the main interface for talking to the time-of-flight camera system. Typically,
     * an instance is created by the `CameraManager` which automatically detects a connected module.
     * The support access levels can be activated by entering the correct code during creation.
     * After creation, the ICameraDevice is in ready state and can be initialized. Before starting
     * capturing an operation mode must be set. The provided examples should give a good overview
     * on how to use this class.
     */
    class DllExport ICameraDevice
    {
    public:
        virtual ~ICameraDevice() = default;

        // ----------------------------------------------------------------------------------------------
        // Level 1: Standard users (Laser Class 1 guaranteed)
        // ----------------------------------------------------------------------------------------------
        /**
         *  LEVEL 1
         *  Initializes the camera device and sets basic settings, this method must be called only once
         *  ideally after the class is created. The settings pointer can also be null which means that a generic
         *  setting is used.
         *
         *  \param settings provides an interface where to get the necessary information from. Currently this
         *         is only required for non-Pico based cameras where calibration information is stored on the
         *         file system.
         *
         * \return SUCCESS if operation mode can be set, MODULE_NOT_SUPPORTED if an unsupported module is used.
         */
        virtual CameraStatus initialize (std::unique_ptr<ICameraSettings> settings = nullptr) = 0;

        /**
         *  LEVEL 1
         *  Get the ID of the camera device
         *
         *  \return Unique ID for the connected camera device as string
         */
        virtual const royale::String &getId() const = 0;

        /*!
         *  LEVEL 1
         *  Sets the operation mode for the camera. If the mode is supported by the connected camera device
         *  SUCCESS will be returned. Please get the list of supported operation modes by calling getOperationModes().
         *
         *  NOTICE: This function must not be called in the data callback - the behavior is undefined,
         *  call it from a different thread instead.
         *
         * \param mode pre-defined operation mode
         *
         * \return SUCCESS if operation mode can be set, MODULE_NOT_SUPPORTED if an unsupported module is used.
         */
        virtual CameraStatus setOperationMode (OperationMode mode) = 0;

        /*!
         *  LEVEL 1
         *  Returns the current operation mode of the camera. If no operation mode was set the OperationMode to be returned
         *  is INVALID_OPERATION_MODE. Please get the list of supported operation modes by calling getOperationModes().
         *
         * \return OPERATIONMODE if operation mode was set, INVALID_OPERATION_MODE if no operationMode was set yet.
         */
        virtual OperationMode getOperationMode () = 0;

        /*!
         *  LEVEL 1
         *  Return all operation modes which are supported by the connected module.
         */
        virtual const Vector<OperationMode> &getOperationModes() const = 0;

        /*!
         *  LEVEL 1
         *  Returns the associated camera type of the connected module. If the module is not
         *  recognized, the camera type UNKNOWN will be returned.
         */
        virtual CameraType getCameraType() const = 0;

        /*!
        *  LEVEL 1
        *  Change the exposure time for the supported operated operation modes. If MANUAL exposure mode of operation is chosen, the user
        *  is able to determine set exposure time manually within the boundaries of the exposure limits of the specific operation mode.
        *  On success the corresponding staus message is returned.
        *  In any other mode of operation the method will return EXPOSURE_MODE_INVALID to indicate incompliance with the
        *  selected exposure mode. If the camera is used in the playback configuration a LOGIC_ERROR is returned instead.
        *
        *  \param exposureTime exposure time in micro seconds
        */
        virtual CameraStatus setExposureTime (uint32_t exposureTime) = 0;

        /*!
        *  LEVEL 1
        *  Change the exposure mode for the supported operated operation modes. If MANUAL exposure mode of operation is chosen, the user
        *  is able to determine set exposure time manually within the boundaries of the exposure limits of the specific operation mode.
        *  In AUTOMATIC mode the optimum exposure settings are determined the system itself.
        *
        *  The default value is MANUAL.
        *
        *  \param exposureMode mode of operation to determine the exposure time
        */
        virtual CameraStatus setExposureMode (ExposureMode exposureMode) = 0;

        /*!
        *  LEVEL 1
        *  Retrieves the current mode of operation for acquisition of the exposure time.
        *
        *  @return current exposure mode
        */
        virtual ExposureMode getExposureMode () const = 0;

        /*!
        *  LEVEL 1
        *  Retrieves the minimum and maximum allowed exposure limits of the specified operation mode.
        *  Can be used to retrieve the allowed operational range for a manual definition of the exposure time.
        */
        virtual const Pair<uint32_t, uint32_t> &getExposureLimits() const = 0;

        /**
         *  LEVEL 1
         *  Once registering the data listener, 3D point cloud data is sent via the callback function.
         *
         *  \param listener interface which needs to implement the callback method
         */
        virtual CameraStatus registerDataListener (IDepthDataListener *listener) = 0;

        /**
         *  LEVEL 1
         *  Unregisters the data depth listener
         */
        virtual CameraStatus unregisterDataListener() = 0;

        /**
         *  LEVEL 1
         *  Starts the video capture mode (free-running), based on the specified operation mode.
         *  A listener needs to be registered in order to retrieve the data stream. Either raw data
         *  or processed data can be consumed. If no data listener is registered an error will be returned
         *  and capturing is not started.
         */
        virtual CameraStatus startCapture() = 0;

        /**
         *  LEVEL 1
         *  Stops the video capturing mode. All buffers should be released again by the data listener.
         */
        virtual CameraStatus stopCapture() = 0;

        /**
         *  LEVEL 1
         *  Returns the maximal width supported by the camera device.
         */
        virtual uint16_t getMaxSensorWidth() const = 0;

        /**
         *  LEVEL 1
         *  Returns the maximal width supported by the camera device.
         */
        virtual uint16_t getMaxSensorHeight() const = 0;

        /**
         *  LEVEL 1
         *  Gets the intrinsics of the camera module which are stored in the calibration file
         *
         *  \param param LensParameters is storing all the relevant information (c,f,p,k)
         *
         *  \return CameraStatus
         */
        virtual CameraStatus getLensParameters (LensParameters &param) const = 0;

        /**
         *  LEVEL 1
         *  Returns the information if the camera is properly initialized and a connection is established
         *
         *  @return true if properly set up
         */
        virtual bool isConnected() const = 0;

        /**
         *  LEVEL 1
         *  Returns the information if the camera is currently in capture mode
         *
         *  @return true if camera is in capture mode
         */
        virtual bool isCapturing() const = 0;

        /*!
         *  LEVEL 1
         *  Returns the current camera device access level
         */
        virtual CameraAccessLevel getAccessLevel() const = 0;

        /*!
         *  LEVEL 1
         *  Start recording the raw data stream into a file. The recording will capture
         *  the raw data coming from the imager.
         *
         *  \param fileName full path of target filename (proposed suffix is .rrf)
         *  \param numberOfFrames indicate the maximal number of frames which should be captured (stop will be called
         *                        automatically). If zero (default) is set, recording will happen till stopRecording
         *                        is called.
         */
        virtual CameraStatus startRecording (const royale::String &fileName, uint32_t numberOfFrames = 0) = 0;

        /*!
         *  LEVEL 1
         *  Stop recording the raw data stream into a file. After the recording is stopped
         *  the file is available on the file system.
         */
        virtual CameraStatus stopRecording() = 0;

        /**
         *  LEVEL 1
         *  Once registering a record listener, the listener gets notified once recording has stopped after
         *  specified frames.
         *  \param listener interface which needs to implement the callback method
         */
        virtual CameraStatus registerRecordListener (IRecordStopListener *listener) = 0;

        /**
         *  LEVEL 1
         *  Unregisters the record listener.
         */
        virtual CameraStatus unregisterRecordListener() = 0;

        /*!
        *  LEVEL 1
        *  Once registering the exposure listener, new exposure values calculated by the processing are sent
        *  to the listener.
        *  \param listener interface which needs to implement the callback method
        */
        virtual CameraStatus registerExposureListener (IExposureListener *listener) = 0;

        /*!
        *  LEVEL 1
        *  Unregisters the exposure listener
        */
        virtual CameraStatus unregisterExposureListener() = 0;


        // ----------------------------------------------------------------------------------------------
        // Level 2: Experienced users (Laser Class 1 guaranteed) - activation key required
        // ----------------------------------------------------------------------------------------------

        /*!
        *  LEVEL 2
        *  Change the exposure time of a certain sequence or all sequences. If the index is out of bounds, or the new
        *  exposure time is not supported, an error will be returned. The exposure time can also be altered during capture
        *  mode. The maximal exposure time is defined by the used operation mode.
        *
        *  \param exposureTime exposure time in micro seconds
        *  \param index index of the sequence to change
        */
        virtual CameraStatus setExposureTime (uint32_t exposureTime, uint16_t index) = 0;

        /*!
         *  LEVEL 2
         *  Set/alter processing parameters in order to control the data output. A list of processing flags
         *  is available as an enumeration. The `Variant` data type can take float, int, or bool. Please
         *  make sure to set the proper `Variant` type for the enum.
         */
        virtual CameraStatus setProcessingParameters (const ProcessingParameterVector &parameters) = 0;

        /*!
         *  LEVEL 2
         *  Retrieve the available processing parameters which are used for the calculation.
         */
        virtual CameraStatus getProcessingParameters (ProcessingParameterVector &parameters) = 0;

        /**
         *  LEVEL 2
         *  Once registering extended data listener, extended data is sent via the callback function.
         *  If depth data only is specified, this listener is not called. For this case, please use
         *  the standard depth data listener.
         *
         *  \param listener interface which needs to implement the callback method
         */
        virtual CameraStatus registerDataListenerExtended (IExtendedDataListener *listener) = 0;

        /**
         *  LEVEL 2
         *  Unregisters the data extended listener.
         */
        virtual CameraStatus unregisterDataListenerExtended() = 0;

        /**
         *  LEVEL 2
         *  Set the callback output data type to one type only.
         *
         *  INFO: This method needs to be called before startCapture(). If is is called during the camera
         *  is in capture mode, it will only have effect after the next stop/start sequence.
         */
        virtual CameraStatus setCallbackData (CallbackData cbData) = 0;

        /**
         *  LEVEL 2
         *  Set the callback output data type using a binary combined bit mask of CallbackData.
         *  Example: setCallbackData(CallbackData::Raw | CallbackData:Depth) will deliver depth and
         *  raw data. The default is set to Depth same as for L1.
         *
         *  INFO: This method needs to be called before startCapture(). If is is called during the camera
         *  is in capture mode, it will only have effect after the next stop/start sequence.
         */
        virtual CameraStatus setCallbackData (uint16_t cbData) = 0;

        /**
        *  LEVEL 2
        *  Loads a different calibration from a file. This calibration data will also be used
        *  by the processing!
        *
        *  \param filename name of the calibration file which should be loaded
        *
        *  \return CameraStatus
        */
        virtual CameraStatus setCalibrationData (const String &filename) = 0;

        /**
        *  LEVEL 2
        *  Loads a different calibration from a given Vector.This calibration data will also be used
        *  by the processing!
        *
        *  \param data calibration data which should be used
        *
        *  \return CameraStatus
        */
        virtual CameraStatus setCalibrationData (const Vector<uint8_t> &data) = 0;

        /**
        *  LEVEL 2
        *  Retrieves the current calibration data.
        *
        *  \param data Vector which will be filled with the calibration data
        */
        virtual CameraStatus getCalibrationData (Vector<uint8_t> &data) = 0;

        /**
        *  LEVEL 2
        *  Tries to write the current calibration file into the internal flash of the device.
        *  If no flash is found RESOURCE_ERROR is returned. If there are errors during the flash
        *  process it will try to restore the original calibration.
        *
        *  This is not yet implemented for all cameras!
        *
        *  \return CameraStatus
        */
        virtual CameraStatus writeCalibrationToFlash () = 0;

        // ----------------------------------------------------------------------------------------------
        // Level 3: Highly professional users (Laser Class 1 not (!) guaranteed) - activation key required
        // ----------------------------------------------------------------------------------------------

        /**
         *  LEVEL 3
         *  Set the target frame rate of the camera device. The frame rate can be altered during capture mode.
         *  If the frame rate is not supported, this method will return an error.
         *
         *  PLEASE NOTE THAT THIS METHOD DOES NOT GUARANTEE LASER CLASS 1 ANYMORE
         *
         *  \param frameRate frame rate in [FPS].
         *  \return CameraStatus
         */
        virtual CameraStatus setFrameRate (uint16_t frameRate) = 0;

        /**
         *  LEVEL 3
         *  Set the capture mode of the camera device which is represented by the `CameraCaptureMode`.
         *  A camera may support different capture modes. Supported capture modes are exposed by operation
         *  modes which can be altered using `setOperationMode()`. Capture modes can also be altered during
         *  free-running mode.
         *
         *  PLEASE NOTE THAT THIS METHOD DOES NOT GUARANTEE LASER CLASS 1 ANYMORE
         *
         *  \param captureMode user-defined capture mode which specifies the sequence, exposure and frame rate
         *
         *  \return CameraStatus
         */
        virtual CameraStatus setCaptureMode (const CameraCaptureMode &captureMode) = 0;
    };
}