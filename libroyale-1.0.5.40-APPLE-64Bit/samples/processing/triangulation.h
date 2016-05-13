#ifndef _TRIANGULATION_H
#define _TRIANGULATION_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>

void Triangulation(const pcl::PointCloud<pcl::PointXYZ> & output)
{
	Timer t;
	t.StartCounter();
    std::cerr << "Triangulation starts :" << std::endl;
    
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(output));
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (50);
    n.compute (*normals);
    
    
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

	//pcl::io::savePLYFile("E:/PCL/data/final_after_computing_normal.ply", *cloud_with_normals);
	//std::cout << "ply final_after_computing_normal finished..." << std::endl;
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

   
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.1);
    
    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (5000); // 4000 won't work
    gp3.setMaximumSurfaceAngle(M_PI/2); // 180 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    
    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

	std::cerr << "Triangulation finished!!!" << std::endl;
	std::cout << "Triangle takes " << t.GetCounter() /1000 << std::endl;
    
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    
    pcl::io::savePLYFile("E:/PCL/data/final.ply", triangles);
    pcl::io::saveVTKFile("E:/PCL/data/final.vtk", triangles);

}


#endif /* end of include guard: _TRIANGULATION_H */
