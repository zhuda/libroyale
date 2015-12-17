#include <royale/CameraManager.hpp>
#include <royale/Vector.hpp>
#include <royale/String.hpp>
#include <iostream>
#include <thread>
#include <chrono>

using namespace royale;
using namespace std;

class MyListener : public IDepthDataListener
{
    void onNewData (const DepthData *data)
    {
        /* Demonstration of how to retrieve exposureTimes
         * There might be different ExposureTimes per RawFrameSet resulting in a vector of
         * exposureTimes, while however the last one is fixed and purely provided for further
         * reference.
         */
        auto sampleVector (data->exposureTimes);

        if (sampleVector.size() > 0)
        {
            cout << "ExposureTimes #1: ";
            for (unsigned int i = 0; i < sampleVector.size(); ++i)
            {
                cout << sampleVector.at (i);
                if (i + 1 < sampleVector.size())
                {
                    cout << ", ";
                }
            }
            cout << endl;
        }

        // make sure that you either process fast or copy the data
        // once this method is left, the data point will be invalid!
    }
};

int main()
{
    // this represents the main camera device object
    std::unique_ptr<ICameraDevice> cameraDevice;

    // the camera manager will query for a connected camera
    {
        CameraManager manager;

        auto camlist = manager.getConnectedCameraList();
        cout << "Detected " << camlist.size() << " camera(s)." << endl;
        cout << "CamID for first device: " << camlist.at (0).c_str() << " with a length of (" << camlist.at (0).length() << ")" << endl;

        if (!camlist.empty())
        {
            cameraDevice = manager.createCamera (camlist[0]);
        }
    }
    // the camera device is now available and CameraManager can be deallocated here

    if (cameraDevice == nullptr)
    {
        cerr << "Cannot create the camera device" << endl;
        return 1;
    }

    // retrieve current operation mode;
    cout << "Current operationMode: " << royale::getOperationModeName (cameraDevice->getOperationMode()) << endl;

    // IMPORTANT: call the initialize method before working with the camera device
    if (cameraDevice->initialize() != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device" << endl;
        return 1;
    }

    if (cameraDevice->getOperationModes().empty())
    {
        cerr << "No operation modes is available" << endl;
        return 1;
    }

    // register a data listener
    MyListener listener;
    cameraDevice->registerDataListener (&listener);

    // set an operation mode
    cameraDevice->setOperationMode (cameraDevice->getOperationModes() [0]);

    // start capture mode
    cameraDevice->startCapture();

    // retrieve current operation mode;
    cout << "New operationMode: " << royale::getOperationModeName (cameraDevice->getOperationMode()).c_str() << endl;

    // let the camera capture for some time
    std::this_thread::sleep_for (std::chrono::seconds (30));

    // change the exposure time (limited by the used operation mode [microseconds]
    if (cameraDevice->setExposureTime (200) != CameraStatus::SUCCESS)
    {
        cerr << "Cannot set exposure time" << endl;
    }
    cout << "Changed exposure time to 200 microseconds ..." << endl;

    // let the camera capture for some time
    std::this_thread::sleep_for (std::chrono::seconds (30));

    // stop capture mode
    cameraDevice->stopCapture();

    return 0;
}
