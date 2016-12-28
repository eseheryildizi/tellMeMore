/*
 * ircp_loop.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: baris
 */

#include <pc_segmentation.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <utils.hpp>

#include <k2g.h>
//#include <k2pcl_light.h>

//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <signal.h>

#define KINECT2

//pcl::PointCloud<PointT>::ConstPtr prev_cloud;
//pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr prev_cloud;
pcl::PointCloud<PointT>::Ptr prev_cloud;
boost::mutex cloud_mutex;
boost::mutex imageName_mutex;
bool writePCD2File = false;
char imageName[100];
bool gotFirst = false;

int
main (int argc, char **argv)
{

  parsedArguments pA;
  if(parseArguments(argc, argv, pA) < 0)
    return 0;

  ridiculous_global_variables::ignore_low_sat       = pA.saturation_hack;
  ridiculous_global_variables::saturation_threshold = pA.saturation_hack_value;
  ridiculous_global_variables::saturation_mapped_value = pA.saturation_mapped_value;
  
    //Helpers

  char charBuff[100];
  char baseName[200];
  char rogueSKillCounter = 48; //0 in ascii

  char tmpS[100];
  bool idleMode = true;

  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;

  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = cloudViewer(cloud_ptr);

  multi_plane_app.initSegmentation(pA.seg_color_ind, pA.ecc_dist_thresh, pA.ecc_color_thresh);
  multi_plane_app.setViewer(viewer);

  float selected_object_features[324];

  float workSpace[] = {-0.4,0.4,-0.4,0.4,0.5,2};//{-1,1,-1,1,0,2};//{-0.3,0.4,-0.25,0.35,0.7,1.1};//Simon on the other side:{-0.1,0.6,-0.4,0.15,0.7,1.1};//{-0.5,0.6,-0.4,0.4,0.4,1.1};
  multi_plane_app.setWorkingVolumeThresholds(workSpace);


//////////////////////////
  //std::cout << "Syntax is: " << argv[0] << " [0|1|2] -processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively\n";
  processor freenectprocessor = OPENGL;

//  if(argc > 1){
//    freenectprocessor = static_cast<processor>(pA.freenectProcessor);//(atoi(argv[1]));
//  }
    
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  K2G k2g(freenectprocessor, true);
  std::cout << "getting cloud" << std::endl;
  cloud = k2g.getCloud();
  prev_cloud = cloud;
  //viewer = cloudViewer(cloud_ptr);


/////////////////////////

  cloud = k2g.updateCloud(cloud);

  //viewer->addPointCloud<PointT> (cloud, "cloud2");

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce(20);

    std::vector<pc_cluster_features> feats;
    int selected_cluster_index = -1;
    cloud = k2g.updateCloud(cloud);
    prev_cloud = cloud;

    selected_cluster_index = multi_plane_app.processOnce(prev_cloud,feats,pA.hue_val,pA.hue_thresh, pA.z_thresh, pA.euc_thresh, pA.pre_proc, pA.seg_color_ind, pA.merge_clusters, pA.displayAllBb);
    if(selected_cluster_index < 0)
      continue;

    float angle = feats[selected_cluster_index].aligned_bounding_box.angle;
    std::cout << "Selected cluster angle (rad, deg): " << angle << " " << angle*180.0/3.14159 << std::endl;
    std::cout << "Selected cluster hue: " << feats[selected_cluster_index].hue << std::endl;
  }

  k2g.shutDown();
  return 0;
}
/*
Eigen::Vector3f z_axis(0,0,1);
Eigen::Vector3f plane_normal(0.0270649, 0.849503, 0.526889);

plane_normal.normalize();

Eigen::Vector3f c = plane_normal.cross(z_axis);//z_axis.cross(plane_normal);
c.normalize();

//std::cout << c << std::endl;
//std::cout << plane_normal << std::endl << std::endl;

double cost  = z_axis.dot(plane_normal);
double half_theta = acos(cost)/2;
double sin_ht = sin(half_theta);

std::cout << half_theta*180./3.14159 << std::endl;
std::cout << c[0] << " " << c[1] << " " << c[2] << " " << std::endl;

Eigen::Quaternion<float> q(cos(half_theta), c[0]*sin_ht, c[1]*sin_ht, c[2]*sin_ht);

Eigen::Matrix3f R = q.toRotationMatrix();
Eigen::Affine3f T = Eigen::Affine3f::Identity();
T.rotate(R);

std::cout << q.w() << " " << q.x() << " " <<  q.y() << " " << q.z() << " " << std::endl;

//pcl::transformPointCloud (*cloud_projected, *cloud_transformed, T);
pcl::transformPointCloud (*cloud, *prev_cloud, T);
//viewer->updatePointCloud<PointT> (prev_cloud, "cloud");
 * */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    Eigen::Vector3f z_axis(0,0,1);
    Eigen::Vector3f plane_normal(0.0270649, 0.849503, 0.526889);

    plane_normal.normalize();

    Eigen::Vector3f c = plane_normal.cross(z_axis);//z_axis.cross(plane_normal);
    c.normalize();


    double cost  = z_axis.dot(plane_normal);
    double half_theta = acos(cost)/2;
    double sin_ht = sin(half_theta);

    //std::cout << half_theta*180./3.14159 << std::endl;
    //std::cout << c[0] << " " << c[1] << " " << c[2] << " " << std::endl;

    Eigen::Quaternion<float> q(cos(half_theta), c[0]*sin_ht, c[1]*sin_ht, c[2]*sin_ht);

    Eigen::Matrix3f R = q.toRotationMatrix();
    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    T.rotate(R);

    std::cout << q.w() << " " << q.x() << " " <<  q.y() << " " << q.z() << " " << std::endl;

    pcl::transformPointCloud (*cloud, *prev_cloud, T);
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
