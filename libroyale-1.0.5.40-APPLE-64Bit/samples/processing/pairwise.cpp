#include "pairwise.h"
#include "filter.h"
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/elch.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;



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
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample/* = false*/)
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
    reg.setMaxCorrespondenceDistance (1.0);
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


void PairWise (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs, pcl::PointCloud<pcl::PointXYZ>& output)
{
    // Create a PCLVisualizer object
    
    PointCloud::Ptr result (new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
    
    target.reset(inputs[0].get()); 
    
    for (size_t i = 1; i < inputs.size(); ++i)
    {
        //target.reset(new PointCloud(inputs[i-1]));
        source = inputs[i];
        
        // Add visualization data
        //showCloudsLeft(source, target);
		std::cout << "=====================================================" << std::endl;
        std::cout << "Pairwise for " << i << "/" << inputs.size() << std::endl;
		std::cout << "=====================================================" << std::endl;

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
}

void IncrementalPairwise(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs, 
						 pcl::PointCloud<pcl::PointXYZ>& output,
						 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& outputs,
						 Eigen::Matrix4f &loop_transform)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>::Ptr icp (new pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>);
    icp->setMaxCorrespondenceDistance (0.05);
    icp->setMaximumIterations (20);
    // Set the transformation epsilon (criterion 2)
    //icp->setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    //icp->setEuclideanFitnessEpsilon (1);
	icp->setInputTarget(inputs[0]);
	//icp->setUseReciprocalCorrespondences(true);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	//icp->setSearchMethodSource(tree);
	//icp->setSearchMethodTarget(tree);
    
    pcl::registration::IncrementalRegistration<pcl::PointXYZ> iicp;
    iicp.setRegistration (icp);
       
	//pcl::PointCloud<pcl::PointXYZ>::Ptr newTarget;
    for (size_t i = 0; i < inputs.size(); ++i)
    {        
        bool rtn = iicp.registerCloud (inputs[i]);
        if(rtn)
        {
            std::cerr << "converged :" << i + 1 << std::endl;
            
            //pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PointCloud<pcl::PointXYZ> tmp;
            pcl::transformPointCloud (*inputs[i], tmp, iicp.getAbsoluteTransform ());
            
            output += tmp;

			pcl::PointCloud<pcl::PointXYZ>::Ptr element(new pcl::PointCloud<pcl::PointXYZ> (tmp));
			outputs.push_back(element);

			if (i == inputs.size() - 1)
			{
				loop_transform = iicp.getAbsoluteTransform();
			}

			//newTarget.reset(new pcl::PointCloud<pcl::PointXYZ>(output));
			//icp->setInputTarget(newTarget);
        }
    }
    
	std::cout << "Pairwise finished!!!!" << std::endl;

 //   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr (new pcl::PointCloud<pcl::PointXYZ>(output));
 //   viewer = simpleVis(output_ptr);
 //   while (!viewer->wasStopped ())
 //   {
 //       viewer->spinOnce (60000);
 //       viewer->close();
 //       boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 //   }
}


void GetCorrespondence(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
	pcl::Correspondences &correspondences)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> ce;
	ce.setInputTarget(target);
	ce.setInputSource(source);
	ce.determineReciprocalCorrespondences(correspondences);
}

void Lum(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs, pcl::PointCloud<pcl::PointXYZ>& output)
{
	size_t size = inputs.size();
	pcl::registration::LUM<pcl::PointXYZ> lum;
	// Add point clouds as vertices to the SLAM graph
	for(size_t i = 0; i < size; i ++)
	{
		lum.addPointCloud (inputs[i]);
	}

	for(size_t j = 0; j < size; j++)
	{
		pcl::Correspondences correspondences;
		size_t index2 = ((j+1) >= size) ? 0 : j+1;
		GetCorrespondence(inputs[j], inputs[index2], correspondences);
		boost::shared_ptr<pcl::Correspondences> spCE(new pcl::Correspondences(correspondences));
		lum.setCorrespondences(j, index2, spCE);
	}

	// Change the computation parameters
	//lum.setMaxIterations (5);
	//lum.setConvergenceThreshold (0.0);
	// Perform the actual LUM computation
	lum.compute ();
	// Return the concatenated point cloud result
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = lum.getConcatenatedCloud ();
	output = *cloud_out;
}

void ELCH(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs, pcl::PointCloud<pcl::PointXYZ>& output, const Eigen::Matrix4f &loop_transform)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>::Ptr icp (new pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>);
	icp->setMaxCorrespondenceDistance (0.05);
	icp->setRANSACOutlierRejectionThreshold(0.05f); 
	icp->setMaximumIterations (50);
	// Set the transformation epsilon (criterion 2)
	//icp->setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	//icp->setEuclideanFitnessEpsilon (1);
	//icp->setInputTarget(inputs[0]);
	//icp->setUseReciprocalCorrespondences(true);

	//typedef boost::adjacency_list<
	//	boost::listS, boost::eigen_vecS, boost::undirectedS,
	//	Vertex,
	//	boost::no_property>
	//	LoopGraph;

	//typedef boost::shared_ptr< LoopGraph > LoopGraphPtr;

	pcl::registration::ELCH<pcl::PointXYZ> elch;
	boost::graph_traits<pcl::registration::ELCH<pcl::PointXYZ>::LoopGraph>::vertex_descriptor start, end;
	size_t size = inputs.size();
	for(size_t i = 0; i < inputs.size(); i ++)
	{
		elch.addPointCloud (inputs[i]);
	}
	elch.setReg(icp);
	elch.setLoopStart(0);
	elch.setLoopEnd(size - 1);
	elch.setLoopTransform(loop_transform);

	//for(size_t i = 0; i < 50; i ++)
	{
		//std::cout << "elch iteration ... " << i << std::endl;
		elch.compute();
	}
	
	pcl::registration::ELCH<pcl::PointXYZ>::LoopGraphPtr g = elch.getLoopGraph(); 
	for (size_t i = 0; i < boost::num_vertices (*g); i++) 
	{ 
		output += *(*g)[i].cloud; 
	}

	//std::cout << "ELCH results " << std::endl;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr (new pcl::PointCloud<pcl::PointXYZ>(output));
	//viewer = simpleVis(output_ptr);
	//while (!viewer->wasStopped ())
	//{
	//	viewer->spinOnce (60000);
	//	viewer->close();
	//	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	//}
}