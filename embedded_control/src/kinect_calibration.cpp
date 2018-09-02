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
rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::fromPCLPointCloud2(pcl_pc,*cloud);
  
 
    viewer = rgbVis(cloud);
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
