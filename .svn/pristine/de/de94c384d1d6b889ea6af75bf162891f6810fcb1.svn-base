/*
 * ircp_loop.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: baris
 */

#include<ircpHandler.hpp>
#include<pc_segmentation.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <utils.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions2.hpp>

#include <signal.h>

pcl::PointCloud<PointT>::ConstPtr prev_cloud;
boost::mutex cloud_mutex;
boost::mutex imageName_mutex;
bool writePCD2File = false;
char imageName[100];
bool gotFirst = false;

// -ur 1 -rt asus_filtered -dt 0.03 -ct 15 -t 14 -e 0.1 -v 40 -b 0
// -ur 1 -rt asus_filtered -dt 0.03 -ct 15 -t 14 -e 0.1 -v 40 -b 0 -sh 0
// -ur 1 -rt asus_filtered -dt 0.03 -ct 15 -t 14 -e 0.1 -v 200 -sh 1 -st 0.2 -sv 200

void
cloud_cb_ros_ (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PCLPointCloud2 pcl_pc;

  if (msg->height == 1){
         sensor_msgs::PointCloud2 new_cloud_msg;
         new_cloud_msg.header = msg->header;
         new_cloud_msg.height = 480;
         new_cloud_msg.width = 640;
         new_cloud_msg.fields = msg->fields;
         new_cloud_msg.is_bigendian = msg->is_bigendian;
         new_cloud_msg.point_step = msg->point_step;
         new_cloud_msg.row_step = 20480;
         new_cloud_msg.data = msg->data;
         new_cloud_msg.is_dense = msg->is_dense;

         pcl_conversions::toPCL(new_cloud_msg, pcl_pc);
  }
  else
    pcl_conversions::toPCL(*msg, pcl_pc);

  pcl::PointCloud<PointT> cloud;
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  //std::cout << cloud.height << std::endl;
    //pcl::PCLPointCloud2 asd = *msg;

    //pcl::fromPCLPointCloud2(asd, *cloud);

    cloud_mutex.lock ();
    prev_cloud = cloud.makeShared();
    if(writePCD2File)
    {
      pcl::PointCloud<PointT>::Ptr saved_cloud(new pcl::PointCloud<PointT>(*prev_cloud));
      std::cout << imageName << std::endl;
      cloud_mutex.unlock ();
      imageName_mutex.lock();
      pcl::io::savePCDFile(imageName, *saved_cloud);
      imageName_mutex.unlock();
      writePCD2File = false;
    }
    else
      cloud_mutex.unlock ();

    gotFirst = true;
}

void
cloud_cb_direct_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    cloud_mutex.lock ();
    prev_cloud = cloud;
    if(writePCD2File)
    {
      pcl::PointCloud<PointT>::Ptr saved_cloud(new pcl::PointCloud<PointT>(*prev_cloud));
      std::cout << imageName << std::endl;
      cloud_mutex.unlock ();
      imageName_mutex.lock();
      pcl::io::savePCDFile(imageName, *saved_cloud);
      imageName_mutex.unlock();
      writePCD2File = false;
    }
    else
      cloud_mutex.unlock ();

    gotFirst = true;
}

int
main (int argc, char **argv)
{

  parsedArguments pA;
  if(parseArguments(argc, argv, pA) < 0)
    return 0;

  ridiculous_global_variables::ignore_low_sat       = pA.saturation_hack;
  ridiculous_global_variables::saturation_threshold = pA.saturation_hack_value;
  ridiculous_global_variables::saturation_mapped_value = pA.saturation_mapped_value;

  //it is easy to add new arguments by using the value2parse class and pA2.values.push_back(...);
  //however, you have to now how to use the value later*/

  //Helpers

  char charBuff[100];
  char baseName[200];
  char rogueSKillCounter = 48; //0 in ascii

  std::stringstream skillName;
  skillName << "skill" << rogueSKillCounter;
  ircpHandler::globalSkillInfo.skillName = skillName.str();

  //Demo information
  //int demoCount = 0;
  std::vector<int> numberOfKeyframes;
  //int kfCount = 0;
  int numFramesPerKF = 5; //not being used right now

  //TODO: decide if you want to wait for c6 to start everything

  //std::vector<objectFeatures> outFeatures;

  char tmpS[100];
  bool idleMode = true;

  resetServiceRequest(ircpHandler::globalRequests);

  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;

  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = cloudViewer(cloud_ptr);

  multi_plane_app.initSegmentation(pA.seg_color_ind, pA.ecc_dist_thresh, pA.ecc_color_thresh);
  multi_plane_app.setViewer(viewer);


  float selected_object_features[324];

  float workSpace[] = {-0.6,0.6,-0.5,0.5,0.4,1.1};//{-0.5,0.6,-0.4,0.4,0.4,1.1};
  multi_plane_app.setWorkingVolumeThresholds(workSpace);

  pcl::Grabber* interface;
  ros::NodeHandle *nh;
  ros::Subscriber sub;
  
  if(pA.use_ros)
  {
      std::cout << "Using ros" << std::endl;
      ros::init(argc, argv, "pc_segmentation",ros::init_options::NoSigintHandler);
      nh = new ros::NodeHandle();
      sub = nh->subscribe<sensor_msgs::PointCloud2>(pA.ros_topic, 1, cloud_cb_ros_);
  }
  else
  {
      std::cout << "Using the device" << std::endl;
      interface = new pcl::OpenNIGrabber ();
      boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&cloud_cb_direct_,_1);
      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();
  }
  
  //multi_plane_app.verbose = false;
 
  /*ofstream myfile;
  myfile.open ("centroids.txt");*/
  char filename[1000];
 
  for(int i = 0; i < 1000; i++)
  {
    if(pA.use_ros)
    {
	  ros::spinOnce();
    }
    viewer->spinOnce(20);
    if(!gotFirst)
      continue;
    std::vector<pc_cluster_features> feats;
    int selected_cluster_index = -1;
    if(cloud_mutex.try_lock ())
    {
      selected_cluster_index = multi_plane_app.run3(prev_cloud,feats,pA.hue_val,pA.hue_thresh, pA.z_thresh, pA.euc_thresh, pA.pre_proc, pA.seg_color_ind, pA.merge_clusters, pA.displayAllBb);
      cloud_mutex.unlock();

      /*the z_thresh may result in wrong cluster to be selected. it might be a good idea to do
      * some sort of mean shift tracking, or selecting the biggest amongst the candidates (vs choosing the most similar color)
      * or sending all the plausible ones to c6 and decide there
      */
    }

    if(selected_cluster_index < 0)
    {
	  std::cout << "No cluster for iteration: " << i << std::endl;
      continue;
    }

    /*float angle = feats[selected_cluster_index].aligned_bounding_box.angle;
    std::cout << "Selected cluster angle (rad, deg): " << angle << " " << angle*180.0/3.14159 << std::endl;
    std::cout << "Selected cluster hue: " << feats[selected_cluster_index].hue << std::endl;*/

    //save this guy
    sprintf(filename,"feature_%d.csv",i);
    feats[selected_cluster_index].write2file(filename);
  }

    
  if(!pA.use_ros)
  {
    interface->stop ();
    delete interface;
  }
  else
  {
    delete nh;
    
  //myfile.close();
    ros::shutdown();
  }
  return 0;
}
