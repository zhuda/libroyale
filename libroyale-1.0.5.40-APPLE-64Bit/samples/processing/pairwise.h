#ifndef _PAIRWISE_H
#define _PAIRWISE_H
/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/incremental_icp.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "constant.h"

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;

//convenient structure to handle our pointclouds
struct PCD
{
    PointCloud::Ptr cloud;
    std::string f_name;
    
    PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
    bool operator () (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }
    
    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);
        
        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }
    
    
    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
    
    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (50);
    
    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);
    
    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
    
    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);
    
    //
    // Align
    pcl::IterativeClosestPoint<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.1);
    // Set the point representation
    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
    
    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);
    
    
    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, SourceToTarget;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);
    for (int i = 0; i < 30; ++i)
    {
        PCL_INFO ("Iteration Nr. %d.\n", i);
        
        // save cloud for visualization purpose
        points_with_normals_src = reg_result;
        
        // Estimate
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);
        
        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;
        
        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
        
        prev = reg.getLastIncrementalTransformation ();
    }
    
    //
    // Get the transformation from target to source
    SourceToTarget = Ti;
    
    //
    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_src, *output, SourceToTarget);
    
    //add the source to the transformed target
    *output += *cloud_tgt;
    
    final_transform = SourceToTarget;
}


void PairWise (std::vector<pcl::PointCloud<pcl::PointXYZ> > & inputs, pcl::PointCloud<pcl::PointXYZ>& output)
{
    // Create a PCLVisualizer object
    
    PointCloud::Ptr result (new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
    
    target.reset(new PointCloud(inputs[0]));
    
    for (size_t i = 1; i < inputs.size(); ++i)
    {
        //target.reset(new PointCloud(inputs[i-1]));
        source.reset(new PointCloud(inputs[i]));
        
        // Add visualization data
        //showCloudsLeft(source, target);
        
        PointCloud::Ptr output (new PointCloud);
        // PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
        pairAlign (source, target, output, pairTransform, false);
        
        //transform current pair into the global transform
        pcl::transformPointCloud (*output, *result, GlobalTransform);
        
        //update the global transform
        GlobalTransform = GlobalTransform * pairTransform;
        
        target.reset(new PointCloud(*result));
    }
    
    output = *target;
    
    //save aligned pair, transformed into the first cloud's frame
    //    std::stringstream ss;
    //    ss << datafolder << "/" << "pairwised.pcd";
    //    pcl::io::savePCDFile (ss.str (), *result, false);
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(target);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void IncrementalPairwise(const std::vector<pcl::PointCloud<pcl::PointXYZ> > & inputs, pcl::PointCloud<pcl::PointXYZ>& output)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>::Ptr icp (new pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>);
    icp->setMaxCorrespondenceDistance (0.05);
    icp->setMaximumIterations (10);
    icp->setInputTarget(boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> >(inputs[0]));
    
    pcl::registration::IncrementalICP<pcl::PointXYZ> iicp;
    iicp.setICP (icp);
    
    PointCloud::Ptr target;
    target.reset(new PointCloud(inputs[0]));
    
    for (size_t i = 0; i < inputs.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(inputs[i]));
        
        bool rtn = iicp.registerCloud (cloud);
        if(rtn)
        {
            std::cerr << "converged :" << i << std::endl;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
            
            pcl::transformPointCloud (*cloud, *tmp, iicp.getAbsoluteTransform ());
            
            *target += *tmp;
        }
    }
    
    output = *target;
   
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(target);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (60000);
        viewer->close();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

#endif /* end of include guard: _PAIRWISE_H */
