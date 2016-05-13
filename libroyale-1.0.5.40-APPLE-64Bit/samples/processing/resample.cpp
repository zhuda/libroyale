#include "resample.h"

#include <vector>
#include <iosfwd>

#include "filter.h"
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include "Timer.h"

void Resample(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & outputs)
{
    // TODO: The normal calculated during resampling filpped for some triangles.
    size_t size = outputs.size();
    for(size_t i = 0; i < size; i ++)
    {
        std::cout << "Start resample data index/total: " << i + 1 << "/" << size << std::endl;
        
        // Create a KD-Tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<pcl::PointXYZ> mls_points;
        
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        
        mls.setComputeNormals (false);
        
        // Set parameters
        mls.setInputCloud (outputs[i]);
        mls.setPolynomialFit (false);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.005); // lower value can help remove far away points.
		//mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
		//mls.setDilationVoxelSize(1);
		//mls.setDilationIterations(20);
        
        // Reconstruct
        mls.process (mls_points);
        outputs[i].reset(new pcl::PointCloud<pcl::PointXYZ>(mls_points));
        
        //std::cout << "After resample data index/total: " << i + 1 << "/" << size << std::endl;
        //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        //viewer = simpleVis(outputs[i]);
        //while (!viewer->wasStopped ())
        //{
        //    viewer->spinOnce (50000);
        //    viewer->close();
        //    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        //}
    }
}
