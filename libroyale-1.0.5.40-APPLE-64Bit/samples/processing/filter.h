#ifndef _FILTER_H
#define _FILTER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>

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
    vg.setLeafSize (0.005f, 0.005f, 0.005f);
    vg.filter (*output);
    std::cout << "PointCloud after voxelGrid_filter has: " << output->points.size ()  << " data points." << std::endl;
    
    //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //    viewer = simpleVis(output);
    //    while (!viewer->wasStopped ())
    //    {
    //        viewer->spinOnce (100);
    //        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    //    }
}

void Filter(const std::vector<std::string>& data_files, std::vector<pcl::PointCloud<pcl::PointXYZ> > & outputs)
{
    for(size_t i = 0; i < data_files.size(); i ++)
    {
        string file = data_files[i];
        // Load input file into a PointCloud<T> with an appropriate type
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        // Load bun0.pcd -- should be available with the PCL archive in test
        pcl::io::loadPCDFile (file.c_str(), *cloud);
        
        std::cout << "Start to filter : " << file.c_str() << std::endl;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove_outerliner (new pcl::PointCloud<pcl::PointXYZ> ());
        
        // remove outerliner
        statistica_outlier_removal(cloud, cloud_remove_outerliner);
        
        // downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsample (new pcl::PointCloud<pcl::PointXYZ> ());
        voxelGrid_filter(cloud_remove_outerliner, cloud_downsample);
        
        outputs.push_back(*cloud_downsample);
    }
}


#endif /* end of include guard: _FILTER_H */
