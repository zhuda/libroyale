#ifndef _RESAMPLE_H
#define _RESAMPLE_H

#include <vector>
#include <iosfwd>

#include "filter.h"

void Resample(pcl::PointCloud<pcl::PointXYZ>& singleFrameData)
{
    // TODO: The normal calculated during resampling filpped for some triangles.
    
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZ> mls_points;
    
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    
    mls.setComputeNormals (false);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (singleFrameData));
    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.007); // lower value can help remove far away points.
    
    // Reconstruct
    mls.process (mls_points);
    
    singleFrameData = mls_points;
    
    std::cout << "after sample......" << std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_resample (new pcl::PointCloud<pcl::PointXYZ> (singleFrameData));
    viewer = simpleVis(cloud_resample);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (1500);
        viewer->close();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


#endif /* end of include guard: _RESAMPLE_H */
