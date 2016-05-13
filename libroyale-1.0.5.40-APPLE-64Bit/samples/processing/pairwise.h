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
#include <pcl/registration/incremental_registration.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);

void PairWise (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs, pcl::PointCloud<pcl::PointXYZ>& output);

void IncrementalPairwise(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs, 
						 pcl::PointCloud<pcl::PointXYZ>& output,
						 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& outputs,
						 Eigen::Matrix4f &loop_transform);

void Lum(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs, pcl::PointCloud<pcl::PointXYZ>& output);

void ELCH(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs, pcl::PointCloud<pcl::PointXYZ>& output,  const Eigen::Matrix4f &loop_transform);


#endif /* end of include guard: _PAIRWISE_H */
