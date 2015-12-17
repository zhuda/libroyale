#include <royale/CameraManager.hpp>
#include <royale/String.hpp>
#include <iostream>

int main()
{
    using namespace royale;
    using namespace std;

    // this represents the main camera device object
    std::unique_ptr<ICameraDevice> cameraDevice;

    // the camera manager will query for a connected camera
    {
        CameraManager manager;

        royale::Vector<royale::String> camlist (manager.getConnectedCameraList());
        cout << "Detected " << camlist.size() << " camera(s)." << endl;

        if (!camlist.empty())
        {
            cameraDevice = manager.createCamera (camlist[0]);
        }
        else
        {
            cerr
                    << "Please make sure that a supported camera is plugged in, all drivers are "
                    << "installed, and you have proper USB permission" << endl;
        }

        camlist.clear();
    }
    // the camera device is now available and CameraManager can be deallocated here

    if (cameraDevice == nullptr)
    {
        cerr << "Cannot create the camera device" << endl;
        return 1;
    }

    // IMPORTANT: call the initialize method before working with the camera device
    auto ret = cameraDevice->initialize();
    if (ret != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device, error code: " << static_cast<int> (ret) << endl;
        return 1;
    }

    // display some information about the connected camera
    cout << "====================================" << endl;
    cout << "        Camera information"           << endl;
    cout << "====================================" << endl;
    cout << "Id:              " << cameraDevice->getId() << endl;
    cout << "Type:            " << (int) cameraDevice->getCameraType() << endl;
    cout << "Width:           " << cameraDevice->getMaxSensorWidth() << endl;
    cout << "Height:          " << cameraDevice->getMaxSensorHeight() << endl;
    cout << "Operation modes: " << cameraDevice->getOperationModes().size() << endl;

    auto mode = cameraDevice->getOperationModes();
    for (size_t i = 0; i < mode.size(); ++i)
    {
        cout << "    " << getOperationModeName (mode[i]) << endl;
    }

    return 0;
}
