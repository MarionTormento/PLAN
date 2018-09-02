#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl_detection/point.h>

ros::Publisher pub;
int N=0;

void 
pp_callback(const pcl::visualization::PointPickingEvent& event, void* 
viewer_void){ 

  pcl_detection::point point;
  std::cerr << "PCPIODIDIDI" << N  << " has been picked" << std::endl; 


  if(event.getPointIndex()!=-1) 
     { 
         float x,y,z; 
         event.getPoint(x,y,z); 
         point.coords.push_back (x);
         point.coords.push_back (y);
         point.coords.push_back (z);
         pub.publish(point);
         std::cerr << "Point " << N  << " has been picked" << std::endl; 
     } 
  N++;

} 

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->registerPointPickingCallback (pp_callback,(void*)&viewer); 
  viewer->initCameraParameters ();

  return (viewer);

}

void
showPCL(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_msg)
{
    // Initialisation of all the methods
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


    // Cloud conversions to the correct format
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromPCLPointCloud2(pcl_pc,*cloud);
  
 
    viewer = simpleVis(cloud);
    int oldN;

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce(100);
      if (N == 10 and oldN!=N){
        std::cout<< "CALIBRATION DONE" << std::endl;
        N = 0;
      }
      else if (N<5 and oldN!=N)
        std::cout<< "Please pick the marker on the LEFT gripper" << std::endl;
      else if (N>=5 and oldN!=N)
        std::cout<< "Please pick the marker on the RIGHT gripper" << std::endl;
      oldN = N;
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "kinect_calibration");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, showPCL);
 
  // Create a ROS publisher for the picked point
  pub = nh.advertise<pcl_detection::point> ("/kinect_calibration/PCLPointXYZ", 1);

  // Spin
  ros::spin ();

}


// #include <ros/ros.h>
// // PCL specific includes
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/filters/extract_indices.h>
// #include "pcl/common/impl/common.hpp"
// #include <pcl_detection/obj.h>
// #include <pcl/filters/project_inliers.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/common/transforms.h>
// #include <string>

// ros::Publisher pub;
// int N=0;

// boost::shared_ptr<pcl::visualization::PCLVisualizer>
// tripleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud3)
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 255); //White
//   viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 0,0,255); //Blue
//   viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color2, "sample cloud2");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample cloud2");
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud3, 255,0,255); //Pink
//   viewer->addPointCloud<pcl::PointXYZ> (cloud3, single_color3, "sample cloud3");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample cloud3");
//   //viewer->addCoordinateSystem (1.0, "global");
//   viewer->initCameraParameters ();
//   return (viewer);

// }

// boost::shared_ptr<pcl::visualization::PCLVisualizer>
// simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 255);
//   viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   //viewer->addCoordinateSystem (1.0, "global");
//   viewer->initCameraParameters ();
//   return (viewer);

// }

// void
// normal_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
// {
//     // Initialisation
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     double meanoflist, stdvoflist;
//     pcl::PointXYZ mean(0.0, 0.0, 0.0);
//     pcl::PointXYZ stdv(0.0, 0.0, 0.0);
//     std::vector<float> cloudX;
//     std::vector<float> cloudY;
//     std::vector<float> cloudZ;

//     for (size_t i = 0; i < cloud_in->points.size (); ++i)
//     {
//       cloudX.push_back (cloud_in->points[i].x);
//       cloudY.push_back (cloud_in->points[i].y);
//       cloudZ.push_back (cloud_in->points[i].z);

//     }

//     pcl::getMeanStd (cloudX, meanoflist, stdvoflist);
//     mean.x = meanoflist;
//     stdv.x = stdvoflist;
//     pcl::getMeanStd  (cloudY, meanoflist, stdvoflist);
//     mean.y = meanoflist;
//     stdv.y = stdvoflist;
//     pcl::getMeanStd  (cloudZ, meanoflist, stdvoflist); 
//     mean.z = meanoflist;
//     stdv.z = stdvoflist;
//     double devfactor = 2;

//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

//     for (int i = 0; i < (*cloud_in).size(); i++)
//     {
//       pcl::PointXYZ pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z), delta;
//       delta.getArray3fMap() = abs(pt.getArray3fMap() - mean.getArray3fMap());
//       if (delta.x < devfactor*stdv.x and delta.y < devfactor*stdv.y and delta.z < devfactor*stdv.z) // e.g. remove all pts below zAvg
//       {
//         inliers->indices.push_back(i);
//       }
//     }
//     extract.setInputCloud(cloud_in);
//     extract.setIndices(inliers);
//     extract.setNegative(false);
//     extract.filter(*cloud_out);  
// }

// int
// isPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane)
// {
//     // Segmentation
//     pcl::SACSegmentation<pcl::PointXYZ> seg;
//     pcl::ProjectInliers<pcl::PointXYZ> proj;
//     pcl::ExtractIndices<pcl::PointXYZ> extract;

//     pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
       
//     // Extract the main plane = plane from the pointcloud
//     seg.setOptimizeCoefficients (true);
//     seg.setModelType (pcl::SACMODEL_PLANE);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     seg.setDistanceThreshold (0.01);
//     seg.setInputCloud (cloud_in);
//     seg.segment (*inliers_plane, *coefficients_plane);

//     // Extract objects point cloud
//     extract.setInputCloud (cloud_in);
//     extract.setIndices (inliers_plane);
//     extract.setNegative (true);
//     extract.filter (*cloud_without);

//     // Project the model inliers    
//     proj.setModelType (pcl::SACMODEL_PLANE);
//     proj.setInputCloud (cloud_in);
//     proj.setIndices (inliers_plane);
//     proj.setModelCoefficients (coefficients_plane);
//     proj.filter (*cloud_plane);


//     // std::cerr << "Ratio of point belonging to the plane model: " << cloud_plane->points.size() << " / " << cloud_in->points.size() << std::endl;

//     return cloud_plane->points.size();
// }

// void
// calibration(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_msg)
// {
//     // Initialisation of all the methods
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


//     // Cloud conversions to the correct format
//     pcl::PCLPointCloud2 pcl_pc;
//     pcl_conversions::toPCL(*cloud_msg,pcl_pc);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
//                                         cloud_floor(new pcl::PointCloud<pcl::PointXYZ>),
//                                         cloud_rest(new pcl::PointCloud<pcl::PointXYZ>),
//                                         cloud_rest2(new pcl::PointCloud<pcl::PointXYZ>),
//                                         cloud_projected(new pcl::PointCloud<pcl::PointXYZ>),
//                                         cloud_calib(new pcl::PointCloud<pcl::PointXYZ>);
    
//     pcl::fromPCLPointCloud2(pcl_pc,*cloud);
  
//     // Segmentation
//     pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
       
//     // Extract the main plane = calib from the pointcloud
//     int calibPts = isPlane(cloud, cloud_rest, cloud_projected);
//     // int calibPts = isPlane(cloud_rest, cloud_rest2, cloud_projected);
//     normal_filter(cloud_projected, cloud_calib);

//     // PCA to find the calib pose and dimensions
//     // Compute principal directions
//     Eigen::Vector4f pcaCentroid;
//     pcl::compute3DCentroid(*cloud_calib, pcaCentroid);
//     Eigen::Matrix3f covariance;
//     computeCovarianceMatrixNormalized(*cloud_calib, pcaCentroid, covariance);
//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
//     Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
//     Eigen::Vector3f y_kinect; y_kinect[0] = 0; y_kinect[1] = 1; y_kinect[2] = 0; 
//     if (eigenVectorsPCA.col(0).dot(y_kinect) > 0)
//     {
//       eigenVectorsPCA.col(0) = - eigenVectorsPCA.col(0);
//     } 
//     eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 

//     // Transform the original cloud to the origin where the principal components correspond to the axes.
//     Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
//     projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
//     projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::transformPointCloud(*cloud_calib, *cloudPointsProjected, projectionTransform);
    
//     // Get the minimum and maximum points of the transformed cloud.
//     pcl::PointXYZ minPoint, maxPoint;
//     pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
//     const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    
//     // Final transform
//     const Eigen::Quaternionf calibQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
//     const Eigen::Vector3f calibTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

//     // Visualisation of the center of the calib
//     float calib_dx, calib_dy;
//     calib_dx = maxPoint.y-minPoint.y;
//     calib_dy = maxPoint.z-minPoint.z;  
  
//     viewer = tripleVis(cloud, cloudPointsProjected, cloud_calib);

//     while (!viewer->wasStopped ())
//     {
//       viewer->spinOnce(100);
//       std::cerr << "dx = " << calib_dx << ", dy = " << calib_dy << std::endl;
//       boost::this_thread::sleep (boost::posix_time::microseconds (100000));

//     }
// }

// int
// main (int argc, char** argv)
// {
//   // Initialize ROS
//   ros::init (argc, argv, "kinect_calibration");
//   ros::NodeHandle nh;

//   // Create a ROS subscriber for the input point cloud
//   ros::Subscriber sub = nh.subscribe ("input", 1, calibration);
 
//   // Create a ROS publisher for the picked point
//   // pub = nh.advertise<pcl_detection::point> ("/kinect_calibration/PCLPointXYZ", 1);

//   // Spin
//   ros::spin ();

// }
