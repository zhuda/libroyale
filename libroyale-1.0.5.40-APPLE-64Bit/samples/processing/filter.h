#ifndef _FILTER_H
#define _FILTER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/point_types.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

void RandomSampleConsensusFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool useSphere, bool usePlane, pcl::PointCloud<pcl::PointXYZ>::Ptr& final);



void statistica_outlier_removal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& output);

void voxelGrid_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& output);

void Filter(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & outputs);

#endif /* end of include guard: _FILTER_H */
