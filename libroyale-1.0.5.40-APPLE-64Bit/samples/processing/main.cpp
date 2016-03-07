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


int
main (int argc, char** argv)
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
    
    Filter(data_files, outputs);
    
    Resample(outputs);
    
    pcl::PointCloud<pcl::PointXYZ> output;
    //PairWise(outputs);
    IncrementalPairwise(outputs, output);
    
    //Triangulation(output);

}
