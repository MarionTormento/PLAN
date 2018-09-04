#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
// #include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
// #include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
// #include <boost/thread/thread.hpp>
// #include <pcl/common/distances.h>
#include "pcl/common/impl/common.hpp"
#include <pcl_detection/box.h>
#include <pcl/filters/project_inliers.h>
// #include <pcl/surface/convex_hull.h>
// #include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/common/transforms.h>

ros::Publisher pub;

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud3)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 0,0,255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color2, "sample cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample cloud2");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud3, 255,0,255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud3, single_color3, "sample cloud3");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample cloud3");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);

}

void
normal_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    // Initialisation
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    double meanoflist, stdvoflist;
    pcl::PointXYZ mean(0.0, 0.0, 0.0);
    pcl::PointXYZ stdv(0.0, 0.0, 0.0);
    std::vector<float> cloudX;
    std::vector<float> cloudY;
    std::vector<float> cloudZ;

    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
      cloudX.push_back (cloud_in->points[i].x);
      cloudY.push_back (cloud_in->points[i].y);
      cloudZ.push_back (cloud_in->points[i].z);

    }

    pcl::getMeanStd (cloudX, meanoflist, stdvoflist);
    mean.x = meanoflist;
    stdv.x = stdvoflist;
    pcl::getMeanStd  (cloudY, meanoflist, stdvoflist);
    mean.y = meanoflist;
    stdv.y = stdvoflist;
    pcl::getMeanStd  (cloudZ, meanoflist, stdvoflist); 
    mean.z = meanoflist;
    stdv.z = stdvoflist;
    double devfactor = 2;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    for (int i = 0; i < (*cloud_in).size(); i++)
    {
      pcl::PointXYZ pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z), delta;
      delta.getArray3fMap() = abs(pt.getArray3fMap() - mean.getArray3fMap());
      if (delta.x < devfactor*stdv.x and delta.y < devfactor*stdv.y and delta.z < devfactor*stdv.z) // e.g. remove all pts below zAvg
      {
        inliers->indices.push_back(i);
      }
    }
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);  
}

void
isCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal)
{    
    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_normal;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
 
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    
    seg_normal.setOptimizeCoefficients (true);
    seg_normal.setModelType (pcl::SACMODEL_CYLINDER);
    seg_normal.setMethodType (pcl::SAC_RANSAC);
    seg_normal.setNormalDistanceWeight (0.05);
    seg_normal.setMaxIterations (10000);
    seg_normal.setDistanceThreshold (0.04);
    seg_normal.setRadiusLimits (0, 0.06);
    seg_normal.setInputCloud (cloud_in);
    seg_normal.setInputNormals (cloud_normal);

    // Obtain the cylinder inliers and coefficients
    seg_normal.segment (*inliers_cylinder, *coefficients_cylinder);
    // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter (*cloud_cylinder);

    std::cerr << "Cylinder ratio " << cloud_cylinder->points.size() << " / " << cloud_in->points.size() << std::endl;
}

void
isPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane)
{
    // Segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
       
    // Extract the main plane = table from the pointcloud
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_in);
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract objects point cloud
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers_plane);
    extract.setNegative (true);
    extract.filter (*cloud_without);

    // Project the model inliers    
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_in);
    proj.setIndices (inliers_plane);
    proj.setModelCoefficients (coefficients_plane);
    proj.filter (*cloud_plane);


    std::cerr << "Plane ratio " << cloud_plane->points.size() << " / " << cloud_in->points.size() << std::endl;
}

void
find_table(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_msg)
{
    // Initialisation of all the methods
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_normal; 


    // Cloud conversions to the correct format
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_table(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_objects(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromPCLPointCloud2(pcl_pc,*cloud);
  
    // Segmentation
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
       
    // Extract the main plane = table from the pointcloud
    isPlane(cloud, cloud_objects, cloud_projected);
    // Clean the data in case random points are in the plane
    normal_filter(cloud_projected, cloud_table);

    // PCA to find the table pose and dimensions
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_table, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud_table, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_table, *cloudPointsProjected, projectionTransform);
    
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    
    // Final transform
    const Eigen::Quaternionf tableQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f tableTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // Visualisation of the center of the table
    float table_dx, table_dy;
    table_dx = maxPoint.y-minPoint.y;
    table_dy = maxPoint.z-minPoint.z;  

    
    // OBJECTS SEGMENTATION
    // From the raw objects pointcloud, only keeps the one that are above the table
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudObjectsProjected (new pcl::PointCloud<pcl::PointXYZ>),
                                        objectsOnTable (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers_objects (new pcl::PointIndices);

    pcl::transformPointCloud(*cloud_objects, *cloudObjectsProjected, projectionTransform);
    for (int i = 0; i < (*cloudObjectsProjected).size(); i++)
    {
      if (cloudObjectsProjected->points[i].y > minPoint.y+0.05
        and cloudObjectsProjected->points[i].y < maxPoint.y-0.05
        and cloudObjectsProjected->points[i].z > minPoint.z+0.05
        and cloudObjectsProjected->points[i].z < maxPoint.z-0.05
        and cloudObjectsProjected->points[i].x > (minPoint.x+maxPoint.x)/2-0.02)
      {
        inliers_objects->indices.push_back(i);

        // cloudObjectsProjected->points[i].y > minPoint.y+0.05
        // and cloudObjectsProjected->points[i].y < maxPoint.y-0.05
        // and cloudObjectsProjected->points[i].z > minPoint.z+0.05
        // and cloudObjectsProjected->points[i].z < maxPoint.z-0.05
        // and cloudObjectsProjected->points[i].x < (minPoint.x+maxPoint.x)/2-0.02
        // and cloudObjectsProjected->points[i].x > (minPoint.x+maxPoint.x)/2-0.50
      }
    }  

    extract.setInputCloud(cloud_objects);
    extract.setIndices(inliers_objects);
    extract.setNegative(false);
    extract.filter(*objectsOnTable);

    // Find the cluster in the pointcloud to have each object separated in one cluster
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (objectsOnTable);
    ec.extract (cluster_indices);

    std::cerr << "Number of objects on the table: " << cluster_indices.size() << std::endl;

    for (int i = 0; i < cluster_indices.size(); i++)
    {
      // Init
      pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>),
                                          cloud_rest (new pcl::PointCloud<pcl::PointXYZ>),
                                          cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_object_normal (new pcl::PointCloud<pcl::Normal>);

      // Copy of the object pcl
      pcl::copyPointCloud<pcl::PointXYZ>(*objectsOnTable, cluster_indices[i], *object);
      std::cerr << "Number of points in cluster " << i << " : " << object->points.size() << std::endl;

      // Estimate point normals
      ne.setSearchMethod (tree);
      ne.setInputCloud (object);
      ne.setKSearch (50);
      ne.compute (*cloud_object_normal);

      // Evaluate the shape of the object
      isCylinder(object, cloud_object_normal);
      isPlane(object, cloud_rest, cloud_plane);

      Eigen::Vector4f pcaCentroid;
      pcl::compute3DCentroid(*object, pcaCentroid);
      Eigen::Matrix3f covariance;
      computeCovarianceMatrixNormalized(*object, pcaCentroid, covariance);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
      Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
      eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 

      // Transform the original cloud to the origin where the principal components correspond to the axes.
      Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
      projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
      projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*object, *cloudPointsProjected, projectionTransform);
      
      // Get the minimum and maximum points of the transformed cloud.
      pcl::PointXYZ minPoint, maxPoint;
      pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
      const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
       
      // Final transform
      const Eigen::Quaternionf cylQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
      const Eigen::Vector3f cylTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

      std::cerr << "maxPoint" << maxPoint.x << " " << maxPoint.y << " " << maxPoint.z << " "<< "minPoint" << minPoint.x << " " << minPoint.y << " " << minPoint.z << std::endl;

    }

    // // OLD METHOD
    pcl::PointCloud<pcl::PointXYZ>::Ptr object1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object2 (new pcl::PointCloud<pcl::PointXYZ>);

    if (cluster_indices.size() >= 2)
    {
        pcl::copyPointCloud<pcl::PointXYZ>(*objectsOnTable, cluster_indices[0], *object1);
        pcl::copyPointCloud<pcl::PointXYZ>(*objectsOnTable, cluster_indices[1], *object2);
    }

    // // Estimate point normals
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_object_normal (new pcl::PointCloud<pcl::Normal>);
    // ne.setSearchMethod (tree);
    // ne.setInputCloud (object1);
    // ne.setKSearch (50);
    // ne.compute (*cloud_object_normal);

    // // Create the segmentation object for cylinder segmentation and set all the parameters
    // pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    
    // seg_normal.setOptimizeCoefficients (true);
    // seg_normal.setModelType (pcl::SACMODEL_CYLINDER);
    // seg_normal.setMethodType (pcl::SAC_RANSAC);
    // seg_normal.setNormalDistanceWeight (0.1);
    // seg_normal.setMaxIterations (10000);
    // seg_normal.setDistanceThreshold (0.02);
    // seg_normal.setRadiusLimits (0, 0.06);
    // seg_normal.setInputCloud (object1);
    // seg_normal.setInputNormals (cloud_object_normal);

    // // Obtain the cylinder inliers and coefficients
    // seg_normal.segment (*inliers_cylinder, *coefficients_cylinder);
    // // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // // Write the cylinder inliers to disk
    // extract.setInputCloud (object1);
    // extract.setIndices (inliers_cylinder);
    // extract.setNegative (false);    
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
    // extract.filter (*cloud_cylinder);


    // std::cerr << "Cylinder coefficients: " << cloud_cylinder->points.size() << " " << object1->points.size() << std::endl;

    // Eigen::Vector4f pcaCentroid2;
    // pcl::compute3DCentroid(*cloud_cylinder, pcaCentroid2);
    // Eigen::Matrix3f covariance2;
    // computeCovarianceMatrixNormalized(*cloud_cylinder, pcaCentroid2, covariance2);
    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver2(covariance2, Eigen::ComputeEigenvectors);
    // Eigen::Matrix3f eigenVectorsPCA2 = eigen_solver2.eigenvectors();
    // eigenVectorsPCA2.col(2) = eigenVectorsPCA2.col(0).cross(eigenVectorsPCA2.col(1)); 

    // // Transform the original cloud to the origin where the principal components correspond to the axes.
    // Eigen::Matrix4f projectionTransform2(Eigen::Matrix4f::Identity());
    // projectionTransform2.block<3,3>(0,0) = eigenVectorsPCA2.transpose();
    // projectionTransform2.block<3,1>(0,3) = -1.f * (projectionTransform2.block<3,3>(0,0) * pcaCentroid2.head<3>());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected2 (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(*cloud_cylinder, *cloudPointsProjected2, projectionTransform2);
    
    // // Get the minimum and maximum points of the transformed cloud.
    // pcl::PointXYZ minPoint2, maxPoint2;
    // pcl::getMinMax3D(*cloudPointsProjected2, minPoint2, maxPoint2);
    // const Eigen::Vector3f meanDiagonal2 = 0.5f*(maxPoint2.getVector3fMap() + minPoint2.getVector3fMap());
    
    // // Final transform
    // const Eigen::Quaternionf cylQuaternion(eigenVectorsPCA2); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    // const Eigen::Vector3f cylTransform = eigenVectorsPCA2 * meanDiagonal2 + pcaCentroid2.head<3>();

    // std::cerr << "maxPoint" << maxPoint2.x << " " << maxPoint2.y << " " << maxPoint2.z << " "<< "minPoint" << minPoint2.x << " " << minPoint2.y << " " << minPoint2.z << std::endl;


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(cloud, cloud_table, objectsOnTable);
    // viewer = simpleVis(cloud, cloud_table, cloud_filtered);

    char isTable; 
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce(100);
       
      // std::cerr << "Is this the table ? (y/n)" << std::endl;
      // std::cin >> isTable;
      
      // if (isTable == 'y')   
      // {
      //   // Create the data and publishes it
      //   pcl_detection::box table;
      //   table.name = "table";
      //   // Center
      //   table.center.push_back (tableTransform[0]);
      //   table.center.push_back (tableTransform[1]);
      //   table.center.push_back (tableTransform[2]);
      //    // Quaternion
      //   table.quaternion.push_back (tableQuaternion.x());
      //   table.quaternion.push_back (tableQuaternion.y());
      //   table.quaternion.push_back (tableQuaternion.z());
      //   table.quaternion.push_back (tableQuaternion.w());

      //   // Dimensions
      //   table.dimensions.push_back (table_dx);
      //   table.dimensions.push_back (table_dy);

      //   // Publish the data
      //   pub.publish (table);
      // }
      //   viewer->close();
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    }
    // }
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud2_to_pcd");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, find_table);
 
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl_detection::box> ("/pcl/table/kinect_frame", 1);

  // Spin
  ros::spin ();

}



    // // Convert to ROS data type
    // pcl::PCLPointCloud2 cloud_final;
    // sensor_msgs::PointCloud2 output;    

    // pcl::toPCLPointCloud2(*cloud_hull, cloud_final);
    // pcl_conversions::fromPCL(cloud_final, output);

    //   // Random sample consensus
    // std::vector<int> inliers;
   //  // created RandomSampleConsensus object and compute the appropriated model
   //  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
   //    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    
   //  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
   //  ransac.setDistanceThreshold (.01);
   //  ransac.computeModel();
   //  ransac.getInliers(inliers);
    
   //  // copies all inliers of the model computed to another PointCloud
   //  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

   //  // Clean the data in case random points are in the plane
   //  double meanoflist, stdvoflist;
   //  pcl::PointXYZ mean(0.0, 0.0, 0.0);
   //  pcl::PointXYZ stdv(0.0, 0.0, 0.0);
   //  std::vector<float> finalX;
   //  std::vector<float> finalY;
   //  std::vector<float> finalZ;

   //  for (size_t i = 0; i < inliers.size (); ++i)
   //  {
   //    finalX.push_back (final->points[i].x);
   //    finalY.push_back (final->points[i].y);
   //    finalZ.push_back (final->points[i].z);

   //  }

   //  pcl::getMeanStd (finalX, meanoflist, stdvoflist);
   //  mean.x = meanoflist;
   //  stdv.x = stdvoflist;
   //  pcl::getMeanStd  (finalY, meanoflist, stdvoflist);
   //  mean.y = meanoflist;
   //  stdv.y = stdvoflist;
   //  pcl::getMeanStd  (finalZ, meanoflist, stdvoflist); 
   //  mean.z = meanoflist;
   //  stdv.z = stdvoflist;
   //  double devfactor = 2;

    // pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // for (int i = 0; i < (*final).size(); i++)
    // {
    //   pcl::PointXYZ pt(final->points[i].x, final->points[i].y, final->points[i].z), delta;
    //   delta.getArray3fMap() = abs(pt.getArray3fMap() - mean.getArray3fMap());
    //   if (delta.x < devfactor*stdv.x and delta.y < devfactor*stdv.y and delta.z < devfactor*stdv.z) // e.g. remove all pts below zAvg
    //   {
    //     inliers2->indices.push_back(i);
    //   }
    // }
    // extract.setInputCloud(final);
    // extract.setIndices(inliers2);
    // extract.setNegative(false);
    // extract.filter(*final);

   //  // Find centroid of the table
   //  pcl::CentroidPoint<pcl::PointXYZ> centroid;
   //  for (size_t i = 0; i < inliers.size (); ++i)
   //  {
   //    centroid.add (final->points[i]);
   //  }

   //  pcl::PointXYZ center1, center0;
   //  centroid.get (center1);
   //  center0.x = 0; center0.y = 0; center0.z=0;

   //  pcl::PointCloud<pcl::PointXYZ> center;
   //  center.push_back (center1);
   //  center.push_back (center0);

   //  // std::cerr << "centroid coordinates" << center1.x << " "
   //  //                                     << center1.y << " "
   //  //                                     << center1.z << std::endl;

   //  // Find dimensions of the objects
   //  pcl::PointXYZ maxP, minP, c1, c2, c3, c4;
    // maxP.x = -100; maxP.y = -100; maxP.z = -100;
   //  minP.x = 100; minP.y = 100; minP.z = 100;
   //  float dx, dy;

   //  for (int i = 0; i < final->points.size (); ++i)
   //  {
   //    if (final->points[i].x > maxP.x)
   //    {
   //      maxP.x = final->points[i].x;
   //    }
   //    if (final->points[i].x < minP.x)
   //    {
   //      minP.x = final->points[i].x;
   //    }
   //    if (final->points[i].y > maxP.y)
   //    {
   //      maxP.y = final->points[i].y;
   //    }
   //    if (final->points[i].y < minP.y)
   //    {
   //      minP.y = final->points[i].y;
   //   }
   //    if (final->points[i].z > maxP.z)
   //    {
   //      maxP.z = final->points[i].z;
   //    }
   //    if (final->points[i].z < minP.z)
   //    {
   //      minP.z = final->points[i].z;
   //   }
   //  }

   //  // Methode fausse !!!

   //  c1.x = minP.x; c1.y = minP.y; c1.z = maxP.z;
   //  c2.x = maxP.x; c2.y = minP.y; c2.z = maxP.z;
   //  c3.x = maxP.x; c3.y = maxP.y; c3.z = minP.z;
   //  c4.x = minP.x; c4.y = maxP.y; c4.z = minP.z;
    
   //  center.push_back (c1);
   //  center.push_back (c2);
   //  center.push_back (c3);
   //  center.push_back (c4);
   //  dx = pcl::euclideanDistance(c1,c2);
   //  dy = pcl::euclideanDistance(c1,c4);

   //  // std::cerr << "dx = " << dx << " dy = " << dy << std::endl;

   //  // creates the visualization object and adds either our orignial cloud or all of the inliers
   //  // depending on the command line arguments specified.
        // // Create a Convex Hull representation of the projected inliers
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ConvexHull<pcl::PointXYZ> chull;
    // chull.setInputCloud (cloud_filtered);
    // chull.reconstruct (*cloud_hull);

// void 
// find_objects(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_table, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_objects)
// {
//   pcl::PCDWriter writer;

//   // Creating the KdTree object for the search method of the extraction
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//   tree->setInputCloud (cloud_objects);

//   std::vector<pcl::PointIndices> cluster_indices;
//   pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//   ec.setClusterTolerance (0.02); // 2cm
//   ec.setMinClusterSize (100);
//   ec.setMaxClusterSize (25000);
//   ec.setSearchMethod (tree);
//   ec.setInputCloud (cloud_objects);
//   ec.extract (cluster_indices);

//   int j = 0;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//   {
//     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//       cloud_cluster->points.push_back (cloud_objects->points[*pit]); //*
//     cloud_cluster->width = cloud_cluster->points.size ();
//     cloud_cluster->height = 1;
//     cloud_cluster->is_dense = true;

//     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//     std::stringstream ss;
//     ss << "cloud_cluster_" << j << ".pcd";
//     writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
//     j++;
//   }

//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//   viewer = simpleVis(cloud, cloud_table, cloud_cluster);
//   while (!viewer->wasStopped ())
//   {
//     viewer->spinOnce (100);
//     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//   }
// }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cstObj1(&object1);
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    // {
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    //     cloud_cluster->points.push_back (objectsOnTable->points[*pit]); //*
    //   cloud_cluster->width = cloud_cluster->points.size ();
    //   cloud_cluster->height = 1;
    //   cloud_cluster->is_dense = true;

    //   std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    // }

    // std::cerr << "point in " << maxPoint.x << " "<< maxPoint.y << " "<< maxPoint.z << " " << std::endl;
    // std::cerr << "point in " << minPoint.x << " "<< minPoint.y << " "<< minPoint.z << " " << std::endl;

    // pcl::PointCloud<pcl::PointXYZ> center;
    // center.push_back (centerTable);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr constCenter(&center);