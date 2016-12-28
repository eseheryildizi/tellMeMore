/*
 * ircp_loop.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: baris
 */

#include <common.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions2.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<PointT>::ConstPtr prev_cloud;
boost::mutex cloud_mutex;
bool gotFirst = false;

void
cloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& msg)
//cloud_cb_ (const pcl::PCLPointCloud2ConstPtr& msg)
{
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*msg, pcl_pc);

  std::cout << msg->height << " " << msg->width << std::endl;

  pcl::PointCloud<PointT> cloud;
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
    //pcl::PCLPointCloud2 asd = *msg;

    //pcl::fromPCLPointCloud2(asd, *cloud);

    cloud_mutex.lock ();
    prev_cloud = cloud.makeShared();
    cloud_mutex.unlock ();

    gotFirst = true;
}

int
main (int argc, char **argv)
{


  ros::init(argc, argv, "pc_segmentation");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("asus_filtered", 1, cloud_cb_);//"/asus/depth_registered/points", 1, cloud_cb_);//


  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (cloud_ptr, 0, 255, 0);
    viewer->addPointCloud<PointT> (cloud_ptr, single_color, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    ros::spinOnce();
    viewer->spinOnce(100);

    if(!gotFirst)
      continue;


    int selected_cluster_index = -1;
    if(cloud_mutex.try_lock ())
    {
        if (!viewer->updatePointCloud<PointT> (prev_cloud, "cloud"))
            viewer->addPointCloud<PointT> (prev_cloud, "cloud");
        cloud_mutex.unlock();
    }
    else
    {
      std::cout << "me no unlock mutex" << std::endl;
    }
  }

  //interface->stop ();
  return 0;
}
