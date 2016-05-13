#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>

#include <boost/thread/thread.hpp>
#include <pcl/io/ply_io.h>

#include <iosfwd>
using namespace std;
#include "Timer.h"

#include "constant.h"
#include "resample.h"
#include "pairwise.h"
#include "triangulation.h"

void SearchDataFiles(std::vector<string>& data_files)
{
#if defined (_WIN32) || defined (_WIN64)
	WIN32_FIND_DATA data;
	string path = datafolder + "\\*.*";
	HANDLE h = FindFirstFile(path.c_str(), &data);
	
	if( h!=INVALID_HANDLE_VALUE ) 
	{
		do
		{
			char*   nPtr = new char [lstrlen( data.cFileName ) + 1];
			for( int i = 0; i < lstrlen( data.cFileName ); i++ )
				nPtr[i] = char( data.cFileName[i] );

			nPtr[lstrlen( data.cFileName )] = '\0';
			cout << nPtr << endl;
			
			string local = datafolder + "\\" + nPtr;
			if(local.find(".pcd") != -1)
				data_files.push_back(local);

		} while(FindNextFile(h,&data));
	} 
	else 
		cout << "Error: No such folder." << endl;
	
	FindClose(h);
#else
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
#endif
}

void LoadData(const std::vector<string>& data_files, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & outputs)
{
    for(size_t i = 0; i < data_files.size(); i ++)
    {
        //if (i % 20 != 0)
        //   continue;
        
        string file = data_files[i];
        // Load input file into a PointCloud<T> with an appropriate type
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        // Load bun0.pcd -- should be available with the PCL archive in test
        pcl::io::loadPCDFile (file.c_str(), *cloud);
        
        std::cout << file.c_str() << " loaded..." << std::endl;
        
        outputs.push_back(cloud);
    }
    
    std::cout << outputs.size() << " files loaded in total........." << std::endl;
}

void TwoStepProcessing()
{
 //   std::vector<string> data_files;
	//SearchDataFiles(data_files);
 //   
 //   size_t clustersize = 30;
 //   size_t loop = data_files.size() / clustersize;
 //   // todo: if there are left frames less than 50, need do one more step.
 //   //size_t remaining = data_files.size() % 50;
 //   for (size_t i = 0; i < loop; i ++)
 //   {
 //       std::vector<string> sub_data_files;
 //       for(size_t j = 0; j < clustersize; j++)
 //       {
 //           sub_data_files.push_back(data_files[i*clustersize + j]);
 //       }
 //       
 //       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outputs;
 //       
 //       LoadData(data_files, outputs);
 //       
 //       Filter(outputs);
 //       
 //       Resample(outputs);
 //       
 //       pcl::PointCloud<pcl::PointXYZ> output;
 //       //PairWise(outputs, output);
 //       IncrementalPairwise(outputs, output);
 //       
 //       char t[32];
 //       sprintf(t, "%04d", i + 1);
 //       std::string s(t);
 //       std::string filename = datafolder2 + "/" + s + ".pcd";
 //       pcl::io::savePCDFile (filename, output, false);
 //   }
 //   
 //   
 //   std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
 //   std::cout << "Now start second level processing...." << std::endl;
 //   
 //   std::vector<string> data_files2;
	//SearchDataFiles(data_files2);

 //   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outputs2;
 //   
 //   LoadData(data_files, outputs2);
 //   
 //   Filter(outputs2);
 //   
 //   Resample(outputs2);
 //   
 //   pcl::PointCloud<pcl::PointXYZ> output2;
 //   //PairWise(outputs, output);
 //   IncrementalPairwise(outputs2, output2);
 //   Triangulation(output2);
}

void OneStepProcessing()
{
    std::vector<string> data_files;
	SearchDataFiles(data_files);
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outputs;
    
    LoadData(data_files, outputs);
    
    Filter(outputs);
    
    Resample(outputs);
		   
    pcl::PointCloud<pcl::PointXYZ> pairwise_output;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outputs2;
	Eigen::Matrix4f loop_transform;
    IncrementalPairwise(outputs, pairwise_output, outputs2,loop_transform);
	
	pcl::PointCloud<pcl::PointXYZ> output;
	//ELCH(outputs2, output, loop_transform);
	Lum(outputs2, output);


	std::cout << "refilter and resample...." << std::endl;
	std::cout << "original data size ..." << output.size() << std::endl;

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> temp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr element (new pcl::PointCloud<pcl::PointXYZ>(output));
	temp.push_back(element);
	Filter(temp);    
    Resample(temp);

	//pcl::io::savePCDFile(datafolder+"\\sub.pcd", *(temp[0]));

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(temp[0]);
	while (!viewer->wasStopped ())
	{
	    viewer->spinOnce (50000);
	    viewer->close();
	    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

    Triangulation(*(temp[0]));
}


int
main (int argc, char** argv)
{
    // the seond step has very poor matching result, could it be the first step introduce noise or the second step parameter not fit.
    //TwoStepProcessing();
	string path = argv[1];
	if(!path.empty())
	{
		datafolder = path;
		std::cout <<"alter data folder to " << path << std::endl;
	}
    
    OneStepProcessing();
}
