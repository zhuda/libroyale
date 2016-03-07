#include <royale/CameraManager.hpp>
#include <royale/Vector.hpp>
#include <royale/String.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <dirent.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace royale;
using namespace std;

string datafolder = "/Users/zhuda/Desktop/github/CamBoard_pico_flexx/libroyale/libroyale-1.0.5.40-APPLE-64Bit/samples/data/";

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
            if(data->points[i].depthConfidence != 0)
            {
                cloud.points[i].x = data->points[i].x;
                cloud.points[i].y = data->points[i].y;
                cloud.points[i].z = data->points[i].z;
            }
        }
        cloud.points.shrink_to_fit();
        
        // Make a unique name
        static int index = 0;
        index ++;
        char t[32];
        sprintf(t, "%04d", index);
        std::string s(t);
        std::string filename = datafolder + s + ".pcd";
        
        typedef std::chrono::high_resolution_clock Clock;
        typedef std::chrono::milliseconds milliseconds;
        Clock::time_point t0 = Clock::now();
        pcl::io::savePCDFileASCII (filename, cloud);
        Clock::time_point t1 = Clock::now();
        milliseconds ms = std::chrono::duration_cast<milliseconds>(t1 - t0);
    

        std::cerr << "Saved " << cloud.points.size () << " data points." << std::endl;
        std::cout << "saving using : " << ms.count() << " ms" << std::endl;



        // make sure that you either process fast or copy the data
        // once this method is left, the data point will be invalid!
    }
};

int main()
{
 
    {
    // These are data types defined in the "dirent" header
    DIR *theFolder = opendir(datafolder.c_str());
    struct dirent *next_file;
    char filepath[256];
    while ( (next_file = readdir(theFolder)) != NULL )
    {
        // build the path for each file in the folder
        sprintf(filepath, "%s/%s", datafolder.c_str(), next_file->d_name);
        remove(filepath);
    }
    closedir(theFolder);
    }
    
    // Clean last data
//    char buffer[256];
//    char *answer = getcwd(buffer, sizeof(buffer));
//    string s_cwd;
//    if (answer)
//    {
//        s_cwd = answer;
//    }
    
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
    cameraDevice->setOperationMode (royale::OperationMode::MODE_9_5FPS_2000);

    // start capture mode
    cameraDevice->startCapture();

    // let the camera capture for some time
    std::this_thread::sleep_for (std::chrono::seconds (10));

    // stop capture mode
    cameraDevice->stopCapture();

    return 0;
}
