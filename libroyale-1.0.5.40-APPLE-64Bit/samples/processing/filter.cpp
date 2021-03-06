#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "filter.h"
#include "Timer.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();
    return (viewer);
}

void RandomSampleConsensusFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool useSphere, bool usePlane, pcl::PointCloud<pcl::PointXYZ>::Ptr& final)
{
    std::vector<int> inliers;
    
    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    if(useSphere)
    {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        ransac.setDistanceThreshold (.5);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }
    else if (usePlane )
    {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
        ransac.setDistanceThreshold (.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }
    
    // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    
    // creates the visualization object and adds either our orignial cloud or all of the inliers
    // depending on the command line arguments specified.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(final);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}



void statistica_outlier_removal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    //sor.setNegative (true); // open this to get outliner
    sor.filter (*output);
    
    std::cerr << "Cloud after statistica_outlier_removal: " << std::endl;
    std::cerr << *output << std::endl;
    
    //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //    viewer = simpleVis(output);
    //    while (!viewer->wasStopped ())
    //    {
    //        viewer->spinOnce (100);
    //        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    //    }
}

void voxelGrid_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    //vg.setLeafSize (0.0025f, 0.0025f, 0.0025f); // GOOD BUT REPORT WARINGS
    vg.setLeafSize (0.005f, 0.005f, 0.005f); // The value would affect the final model's precision, higher value will lead the model be corase.
    
    vg.filter (*output);
    std::cout << "PointCloud after voxelGrid_filter has: " << output->points.size ()  << " data points." << std::endl;
    
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //viewer = simpleVis(output);
    //while (!viewer->wasStopped ())
    //{
    //    viewer->spinOnce (5000);
    //    viewer->close();
    //    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    //}
}

void Filter(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & outputs)
{
    size_t size = outputs.size();
    for(size_t i = 0; i < size; i ++)
    {
        std::cout << "Start to filter : " << i + 1 << "/" << size << std::endl;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove_outerliner (new pcl::PointCloud<pcl::PointXYZ> ());
        
		Timer t;
		//t.StartCounter();
        // remove outerliner
        //statistica_outlier_removal(outputs[i], cloud_remove_outerliner);
		//std::cout << "outlier removal used: " << t.GetCounter() / 1000 << std::endl;
        
        // downsample
		t.StartCounter();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
        voxelGrid_filter(outputs[i], cloud_downsample);
 		std::cout << "voxelGrid_filter used: " << t.GetCounter() / 1000 << std::endl;

        outputs[i] = cloud_downsample;
    }
}
