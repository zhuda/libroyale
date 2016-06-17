#include "pairwise.h"
#include "filter.h"
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/elch.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>
#include "Timer.h"

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

void PreAlignmentAllFrames(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs, pcl::PointCloud<pcl::PointXYZ>& output)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,cloud_source;
    size_t size = inputs.size();
    Eigen::Matrix4f finalTransformation;
    int icp_converged = 0;
    double icp_score = 0.0;
    
    output += *inputs[0];
    
    for(size_t i = 0; i < size - 1; i++)
    {
        pcl::PointCloud<pcl::PointXYZ> align_single_frame_output;

        std::cout << "start to preAlligment " << i << std::endl;
        
        cloud_target = inputs[i];
        cloud_source = inputs[(i+1) == size ? 0 : i + 1];
        
        Eigen::Matrix4f localTransform = PreAlignment(cloud_target,
                                                      cloud_source,
                                                      &icp_converged,
                                                      &icp_score,
                                                      align_single_frame_output);
        //finalTransformation = finalTransformation * localTransform;
        
        //pcl::PointCloud<pcl::PointXYZ> tmp;
        //pcl::transformPointCloud (*cloud_source, tmp, finalTransformation);
        
        output += align_single_frame_output;
    }
}

Eigen::Matrix4f PreAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_target,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_source,
                             int    *icp_converged,
                             double *icp_score,
                             pcl::PointCloud<pcl::PointXYZ> & icp_registration_output,
                             int full_registration,
                             float  sac_ia_min_sample_distance,
                             float  sac_ia_max_correspondence_distance,
                             int    sac_ia_maximum_iterations,
                             double icp_RANSAC_threshold,
                             int    icp_maxICP_iterations,
                             double icp_maxCorrespondence_distance,
                             double icp_EuclideanFitness_epsilon,
                             double icp_Transformation_epsilon,
                             double normals_radius,
                             double fpfh_radius
                             
                             )
{
    Eigen::Matrix4f initialTransformation, finalTransformation;
    
    Timer t;
    
    if(full_registration == 0)
    {
        // 1. STEP ->  rough initial alignment using the feature descriptors (fast point feature histogram descriptors)
        
        t.StartCounter();
        
        
        // TARGET CLOUD FIRST
        // create the normal estimation class (multi-thread compatible, OpenMP), and pass the input dataset to it
        pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> n_target;
        n_target.setInputCloud(cloud_target);
        // create an empty kdtree representation, and pass it to the normal estimation object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_target_normals (new pcl::search::KdTree<pcl::PointXYZ>);
        n_target.setSearchMethod(tree_target_normals);
        // output datasets
        pcl::PointCloud<pcl::Normal>::Ptr normals_target (new pcl::PointCloud<pcl::Normal>);
        // use all neighbors in a sphere of radius
        n_target.setRadiusSearch(normals_radius);
        // compute the normals
        n_target.compute(*normals_target);
        
        std::cout << "target NormalEstimationOMP " << t.GetCounter() << std::endl;
        
        t.StartCounter();
        
        // create the FPFH estimation class, and pass the input dataset + normals to it
        pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> f_target;
        //f_target.setNumberOfThreads(1);
        f_target.setInputCloud(cloud_target);
        f_target.setInputNormals(normals_target);
        // create an empty kdtree representation, and pass it to the FPFH estimation object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_target_fpfh (new pcl::search::KdTree<pcl::PointXYZ>);
        f_target.setSearchMethod(tree_target_fpfh);
        // output datasets
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_target (new pcl::PointCloud<pcl::FPFHSignature33>);
        // use all neighbors in a sphere of radius (has to be larger than normal estimation radius!)
        f_target.setRadiusSearch(fpfh_radius);
        // compute the normals
        f_target.compute(*fpfh_target);
        
        std::cout << "target FPFHEstimation " << t.GetCounter() << std::endl;
        
        t.StartCounter();
        // SOURCE CLOUD SECOND
        // create the normal estimation class (multi-thread compatible, OpenMP), and pass the input dataset to it
        pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> n_source;
        n_source.setInputCloud(cloud_source);
        // create an empty kdtree representation, and pass it to the normal estimation object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_source_normals (new pcl::search::KdTree<pcl::PointXYZ>);
        n_source.setSearchMethod(tree_source_normals);
        // output datasets
        pcl::PointCloud<pcl::Normal>::Ptr normals_source (new pcl::PointCloud<pcl::Normal>);
        // use all neighbors in a sphere of radius
        n_source.setRadiusSearch(normals_radius);
        // compute the normals
        n_source.compute(*normals_source);
        std::cout << "source NormalEstimationOMP " << t.GetCounter() << std::endl;
        
        t.StartCounter();
        // create the FPFH estimation class, and pass the input dataset + normals to it
        pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> f_source;
        f_source.setInputCloud(cloud_source);
        f_source.setInputNormals(normals_source);
        // create an empty kdtree representation, and pass it to the FPFH estimation object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_source_fpfh (new pcl::search::KdTree<pcl::PointXYZ>);
        f_source.setSearchMethod(tree_source_fpfh);
        // output datasets
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source (new pcl::PointCloud<pcl::FPFHSignature33>);
        // use all neighbors in a sphere of radius (has to be larger than normal estimation radius!)
        f_source.setRadiusSearch(fpfh_radius);
        // compute the normals
        f_source.compute(*fpfh_source);
        
        std::cout << "source FPFHEstimation " << t.GetCounter() << std::endl;
        
        t.StartCounter();
        // sample consensus initial alignment registration
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> sac_ia;
        sac_ia.setMinSampleDistance(sac_ia_min_sample_distance);
        sac_ia.setMaxCorrespondenceDistance(sac_ia_max_correspondence_distance);
        sac_ia.setMaximumIterations(sac_ia_maximum_iterations);
        
        // input the datasets
        // target cloud
        sac_ia.setInputTarget(cloud_target);
        sac_ia.setTargetFeatures(fpfh_target);
        // source cloud
        sac_ia.setInputCloud(cloud_source);
        sac_ia.setSourceFeatures(fpfh_source);
        // align the point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr sac_ia_registration (new pcl::PointCloud<pcl::PointXYZ>);
        sac_ia.align(*sac_ia_registration);
        // get the transformation
        initialTransformation = sac_ia.getFinalTransformation();
        std::cout << "SampleConsensusInitialAlignment " << t.GetCounter() << std::endl;
        
        
        
        
        // 2. STEP -> ICP refined alignment
        
        t.StartCounter();
        // create instance of ICP and set the clouds
        pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
        icp.setInputTarget(cloud_target);
        icp.setInputSource(sac_ia_registration);
        // set the parameters
        icp.setRANSACOutlierRejectionThreshold(icp_RANSAC_threshold);
        icp.setMaximumIterations(icp_maxICP_iterations);
        icp.setMaxCorrespondenceDistance(icp_maxCorrespondence_distance);
        icp.setEuclideanFitnessEpsilon(icp_EuclideanFitness_epsilon);
        icp.setTransformationEpsilon(icp_Transformation_epsilon);
        // set resultant cloud
        //pcl::PointCloud<pcl::PointXYZ> icp_registration;
        // align
        icp.align(icp_registration_output);
        // check if converged
        (*icp_converged) = icp.hasConverged();
        // check score
        (*icp_score) = icp.getFitnessScore();
        // get final transformation (consider sac_ia transformation)
        finalTransformation = icp.getFinalTransformation() * initialTransformation;
        std::cout <<"ICP " << t.GetCounter() << std::endl;
         
         
        
    }
    
    // ONLY ICP
    else if (full_registration == 1) {
        // create instance of ICP and set the clouds
        pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
        icp.setInputTarget(cloud_target);
        icp.setInputCloud(cloud_source);
        // set the parameters
        icp.setRANSACOutlierRejectionThreshold(icp_RANSAC_threshold);
        icp.setMaximumIterations(icp_maxICP_iterations);
        icp.setMaxCorrespondenceDistance(icp_maxCorrespondence_distance);
        icp.setEuclideanFitnessEpsilon(icp_EuclideanFitness_epsilon);
        icp.setTransformationEpsilon(icp_Transformation_epsilon);
        // set resultant cloud
        pcl::PointCloud<pcl::PointXYZ> icp_registration;
        // align
        icp.align(icp_registration);
        // check if converged
        (*icp_converged) = icp.hasConverged();
        std::cout << *icp_converged << " " << *icp_score << std::endl;
        // check score
        (*icp_score) = icp.getFitnessScore();
        // get final transformation (consider sac_ia transformation)
        finalTransformation = icp.getFinalTransformation();
    }
    
    // ONLY SAC-IA
    else {
        // 1. STEP ->  rough initial alignment using the feature descriptors (fast point feature histogram descriptors)
        
        // TARGET CLOUD FIRST
        // create the normal estimation class (multi-thread compatible, OpenMP), and pass the input dataset to it
        pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> n_target;
        n_target.setInputCloud(cloud_target);
        // create an empty kdtree representation, and pass it to the normal estimation object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_target_normals (new pcl::search::KdTree<pcl::PointXYZ>);
        n_target.setSearchMethod(tree_target_normals);
        // output datasets
        pcl::PointCloud<pcl::Normal>::Ptr normals_target (new pcl::PointCloud<pcl::Normal>);
        // use all neighbors in a sphere of radius
        n_target.setRadiusSearch(normals_radius);
        // compute the normals
        n_target.compute(*normals_target);
        
        // create the FPFH estimation class, and pass the input dataset + normals to it
        pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> f_target;
        f_target.setInputCloud(cloud_target);
        f_target.setInputNormals(normals_target);
        // create an empty kdtree representation, and pass it to the FPFH estimation object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_target_fpfh (new pcl::search::KdTree<pcl::PointXYZ>);
        f_target.setSearchMethod(tree_target_fpfh);
        // output datasets
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_target (new pcl::PointCloud<pcl::FPFHSignature33>);
        // use all neighbors in a sphere of radius (has to be larger than normal estimation radius!)
        f_target.setRadiusSearch(fpfh_radius);
        // compute the normals
        f_target.compute(*fpfh_target);
        
        // SOURCE CLOUD SECOND
        // create the normal estimation class (multi-thread compatible, OpenMP), and pass the input dataset to it
        pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> n_source;
        n_source.setInputCloud(cloud_source);
        // create an empty kdtree representation, and pass it to the normal estimation object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_source_normals (new pcl::search::KdTree<pcl::PointXYZ>);
        n_source.setSearchMethod(tree_source_normals);
        // output datasets
        pcl::PointCloud<pcl::Normal>::Ptr normals_source (new pcl::PointCloud<pcl::Normal>);
        // use all neighbors in a sphere of radius
        n_source.setRadiusSearch(normals_radius);
        // compute the normals
        n_source.compute(*normals_source);
        
        // create the FPFH estimation class, and pass the input dataset + normals to it
        pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> f_source;
        f_source.setInputCloud(cloud_source);
        f_source.setInputNormals(normals_source);
        // create an empty kdtree representation, and pass it to the FPFH estimation object.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_source_fpfh (new pcl::search::KdTree<pcl::PointXYZ>);
        f_source.setSearchMethod(tree_source_fpfh);
        // output datasets
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source (new pcl::PointCloud<pcl::FPFHSignature33>);
        // use all neighbors in a sphere of radius (has to be larger than normal estimation radius!)
        f_source.setRadiusSearch(fpfh_radius);
        // compute the normals
        f_source.compute(*fpfh_source);
        
        // sample consensus initial alignment registration
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> sac_ia;
        sac_ia.setMinSampleDistance(sac_ia_min_sample_distance);
        sac_ia.setMaxCorrespondenceDistance(sac_ia_max_correspondence_distance);
        sac_ia.setMaximumIterations(sac_ia_maximum_iterations);
        
        // input the datasets
        // target cloud
        sac_ia.setInputTarget(cloud_target);
        sac_ia.setTargetFeatures(fpfh_target);
        // source cloud
        sac_ia.setInputCloud(cloud_source);
        sac_ia.setSourceFeatures(fpfh_source);
        // align the point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr sac_ia_registration (new pcl::PointCloud<pcl::PointXYZ>);
        sac_ia.align(*sac_ia_registration);
        // get the transformation
        initialTransformation = sac_ia.getFinalTransformation();
        finalTransformation = initialTransformation;
    }
    
}

void IncrementalPairwise(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > & inputs,
                         pcl::PointCloud<pcl::PointXYZ>& output,
                         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& outputs,
                         Eigen::Matrix4f &loop_transform)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>::Ptr icp (new pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>);
    icp->setMaxCorrespondenceDistance (0.05);
    icp->setMaximumIterations (20);
    icp->setInputTarget(inputs[0]);
    
    pcl::registration::IncrementalRegistration<pcl::PointXYZ> iicp;
    iicp.setRegistration (icp);
    
    for (size_t i = 0; i < inputs.size(); ++i)
    {
        bool rtn = iicp.registerCloud (inputs[i]);
        if(rtn)
        {
            std::cerr << "converged :" << i + 1 << std::endl;
            
            
            pcl::PointCloud<pcl::PointXYZ> tmp;
            pcl::transformPointCloud (*inputs[i], tmp, iicp.getAbsoluteTransform ());
            
            output += tmp;
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
    icp->setMaxCorrespondenceDistance (0.0);
    icp->setRANSACOutlierRejectionThreshold(0.0);
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