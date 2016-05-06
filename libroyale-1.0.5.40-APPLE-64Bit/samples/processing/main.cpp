#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/ply_io.h>

#include <iosfwd>
using namespace std;

#include "constant.h"
#include "resample.h"
#include "pairwise.h"
#include "triangulation.h"

void LoadData(const std::vector<string>& data_files, std::vector<pcl::PointCloud<pcl::PointXYZ> >& outputs)
{
    for(size_t i = 0; i < data_files.size(); i ++)
    {
//        // Ignore index 1, 3, 5, 7...
//        if ( i % 2 != 0)
//            continue;
        if (i % 4 != 0)
            continue;
        
        string file = data_files[i];
        // Load input file into a PointCloud<T> with an appropriate type
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        // Load bun0.pcd -- should be available with the PCL archive in test
        pcl::io::loadPCDFile (file.c_str(), *cloud);
        
        std::cout << file.c_str() << " loaded..." << std::endl;
        
        outputs.push_back(*cloud);
    }
    
    std::cout << outputs.size() << " files loaded in total........." << std::endl;
}

void TwoStepProcessing()
{
    std::vector<string> data_files;
    {
        // These are data types defined in the "dirent" header
        DIR *theFolder = opendir(datafolder.c_str());
        struct dirent *next_file;
        char filepath[256];
        while ( (next_file = readdir(theFolder)) != NULL )
        {
            if (strcmp(".", next_file->d_name) == 0 || strcmp("..", next_file->d_name) == 0) {
                continue;
            }
            // build the path for each file in the folder
            sprintf(filepath, "%s/%s", datafolder.c_str(), next_file->d_name);
            
            string strFilePath(filepath);
            if (strFilePath.find(".pcd") != std::string::npos)
                data_files.push_back(filepath);
        }
        closedir(theFolder);
    }
    
    size_t clustersize = 30;
    size_t loop = data_files.size() / clustersize;
    // todo: if there are left frames less than 50, need do one more step.
    //size_t remaining = data_files.size() % 50;
    for (size_t i = 0; i < loop; i ++)
    {
        std::vector<string> sub_data_files;
        for(size_t j = 0; j < clustersize; j++)
        {
            sub_data_files.push_back(data_files[i*clustersize + j]);
        }
        
        std::vector<pcl::PointCloud<pcl::PointXYZ> > outputs;
        
        LoadData(data_files, outputs);
        
        Filter(outputs);
        
        Resample(outputs);
        
        pcl::PointCloud<pcl::PointXYZ> output;
        //PairWise(outputs, output);
        IncrementalPairwise(outputs, output);
        
        char t[32];
        sprintf(t, "%04d", i + 1);
        std::string s(t);
        std::string filename = datafolder2 + "/" + s + ".pcd";
        pcl::io::savePCDFile (filename, output, false);
    }
    
    
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "Now start second level processing...." << std::endl;
    
    std::vector<string> data_files2;
    {
        // These are data types defined in the "dirent" header
        DIR *theFolder = opendir(datafolder2.c_str());
        struct dirent *next_file;
        char filepath[256];
        while ( (next_file = readdir(theFolder)) != NULL )
        {
            if (strcmp(".", next_file->d_name) == 0 || strcmp("..", next_file->d_name) == 0) {
                continue;
            }
            // build the path for each file in the folder
            sprintf(filepath, "%s/%s", datafolder2.c_str(), next_file->d_name);
            
            string strFilePath(filepath);
            if (strFilePath.find(".pcd") != std::string::npos)
                data_files2.push_back(filepath);
        }
        closedir(theFolder);
    }
    std::vector<pcl::PointCloud<pcl::PointXYZ> > outputs2;
    
    LoadData(data_files, outputs2);
    
    Filter(outputs2);
    
    Resample(outputs2);
    
    pcl::PointCloud<pcl::PointXYZ> output2;
    //PairWise(outputs, output);
    IncrementalPairwise(outputs2, output2);
    Triangulation(output2, datafolder2);
}

void OneStepProcessing()
{
    std::vector<string> data_files;
    {
        // These are data types defined in the "dirent" header
        DIR *theFolder = opendir(datafolder.c_str());
        struct dirent *next_file;
        char filepath[256];
        while ( (next_file = readdir(theFolder)) != NULL )
        {
            if (strcmp(".", next_file->d_name) == 0 || strcmp("..", next_file->d_name) == 0) {
                continue;
            }
            // build the path for each file in the folder
            sprintf(filepath, "%s/%s", datafolder.c_str(), next_file->d_name);
            
            string strFilePath(filepath);
            if (strFilePath.find(".pcd") != std::string::npos)
                data_files.push_back(filepath);
        }
        closedir(theFolder);
    }
    
    std::vector<pcl::PointCloud<pcl::PointXYZ> > outputs;
    
    LoadData(data_files, outputs);
    
    Filter(outputs);
    
    Resample(outputs);
    
    pcl::PointCloud<pcl::PointXYZ> output;
    IncrementalPairwise(outputs, output);
    
    Triangulation(output, datafolder);
}


int
main (int argc, char** argv)
{
    // the seond step has very poor matching result, could it be the first step introduce noise or the second step parameter not fit.
    //TwoStepProcessing();
    
    OneStepProcessing();
}
