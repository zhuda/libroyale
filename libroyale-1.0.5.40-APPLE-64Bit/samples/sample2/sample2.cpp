#include <royale/CameraManager.hpp>
#include <royale/Vector.hpp>
#include <royale/String.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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
       
 
        pcl::PointCloud<pcl::PointXYZ> cloud;
        
        // Fill in the cloud data
        cloud.width    = data->width;
        cloud.height   = data->height;
        cloud.is_dense = false;
        cloud.points.resize (cloud.width * cloud.height);
        
        for (size_t i = 0; i < data->points.size(); ++i)
        {
            cloud.points[i].x = data->points[i].x;
            cloud.points[i].y = data->points[i].y;
            cloud.points[i].z = data->points[i].z;
        }
        
        // Make a unique name
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        struct tm *p;
        p = localtime(&now);
        p->tm_year = p->tm_year + 1900;
        p->tm_mon = p->tm_mon + 1;
        
        char chTmp[15];
        snprintf(chTmp,sizeof(chTmp),"%04d-%02d-%02d-%02d-%02d-%02d",
                 p->tm_year, p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
        
        std::string filename(chTmp);
        filename += ".pcd";
        
        
        auto begin = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        pcl::io::savePCDFileASCII (filename, cloud);
        auto end = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        double time = difftime(end, begin);
        std::cout << "saving using seconds: " << time << std::endl;
        std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
        



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
