#ifndef _RESAMPLE_H
#define _RESAMPLE_H

#include <vector>
#include <iosfwd>

#include "filter.h"

void Resample(std::vector<pcl::PointCloud<pcl::PointXYZ> > & outputs)
{
    // TODO: The normal calculated during resampling filpped for some triangles.
    size_t size = outputs.size();
    for(size_t i = 0; i < size; i ++)
    {
        std::cout << "Start resample data index/total: " << i << "/" << size << std::endl;
        
        // Create a KD-Tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<pcl::PointXYZ> mls_points;
        
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        
        mls.setComputeNormals (false);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (outputs[i]));
        // Set parameters
        mls.setInputCloud (cloud);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.005);
        
        // Reconstruct
        mls.process (mls_points);
        
        outputs[i] = mls_points;
        
//        std::cout << "after resample" << std::endl;
//        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_resample (new pcl::PointCloud<pcl::PointXYZ> (mls_points));
//        viewer = simpleVis(cloud_resample);
//        while (!viewer->wasStopped ())
//        {
//            viewer->spinOnce (100);
//            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//        }
    }
}


#endif /* end of include guard: _RESAMPLE_H */
