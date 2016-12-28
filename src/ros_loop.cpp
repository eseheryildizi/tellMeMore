/*
 * ircp_loop.cpp
 *
 *  Created on: Apr 14, 2013
 *      Author: baris
 */

#define WORLD_FRAME

#include<ircpHandler.hpp>
#include<pc_segmentation.hpp>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <utils.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_conversions2.hpp>

#include <signal.h>

pcl::PointCloud<PointT>::ConstPtr prev_cloud;
boost::mutex cloud_mutex;
boost::mutex imageName_mutex;
bool writePCD2File = false;
char imageName[100];
bool gotFirst = false;

/*void
cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
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
}*/
void
cloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& msg)
//cloud_cb_ (const pcl::PCLPointCloud2ConstPtr& msg)
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

void mySigintHandler(int sig) {
  ros::shutdown();
}

int
main (int argc, char **argv)
{

  parsedArguments pA;

  if(parseArguments(argc, argv, pA) < 0)
    return 0;

  //IRCP RELATED
  unsigned char module = OVERHEAD_VISION_BASLER_MODULE_ID;
  unsigned char targetModule = BEHAVIOR_MODULE_ID;
  unsigned char robot = SIMCO_ID;//SIMCO_ID;//SIMON_ID;

  ircpHandler myIrcp(0x7f, module, robot);
  myIrcp.initialize();

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

  makeFolder(ircpHandler::globalSkillInfo.dataLocation.c_str());
  ircpHandler::globalSkillInfo.getSubjectFolder(charBuff);
  makeFolder(charBuff);

  //std::vector<objectFeatures> outFeatures;

  char tmpS[100];
  bool idleMode = true;

  resetServiceRequest(ircpHandler::globalRequests);

  //PCL RELATED
  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;

  //pcl::Grabber* interface = new pcl::OpenNIGrabber ();

  //ros related
  ros::init(argc, argv, "pc_segmentation",ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("asus_filtered", 1, cloud_cb_);//"/camera/depth_registered/points", 1, cloud_cb_);//
  //ros::Subscriber sub = nh.subscribe<pcl::PCLPointCloud2>("asus_filtered", 1, cloud_cb_);

  /*boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&cloud_cb_,_1);
  boost::signals2::connection c = interface->registerCallback (f);*/

  signal(SIGINT, mySigintHandler);

  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = cloudViewer(cloud_ptr);

  multi_plane_app.initSegmentation();
  multi_plane_app.setViewer(viewer);

  float workSpace[] = {-0.5,1.5,-0.5,0.5,-1.2,1.2};
  multi_plane_app.setWorkingVolumeThresholds(workSpace);

  resetServiceRequest(ircpHandler::globalRequests);

  float selected_object_features[324];

  //interface->start ();
  while (!viewer->wasStopped ())
  {
    ros::spinOnce();
    viewer->spinOnce(50);
    if(!gotFirst)
      continue;
    std::vector<pc_cluster_features> feats;
    int selected_cluster_index = -1;
    if(cloud_mutex.try_lock ())
    {
      selected_cluster_index = multi_plane_app.processOnce(prev_cloud,feats,pA.hue_val,pA.hue_thresh, pA.z_thresh, true);
      cloud_mutex.unlock();

      /*the z_thresh may result in wrong cluster to be selected. it might be a good idea to do
      * some sort of mean shift tracking, or selecting the biggest amongst the candidates (vs choosing the most similar color)
      * or sending all the plausible ones to c6 and decide there
      */
    }
    else
    {
      std::cout << "me no unlock mutex" << std::endl;
    }

    continue;

    if(selected_cluster_index < 0)
      continue;

    float angle = feats[selected_cluster_index].aligned_bounding_box.angle;
    std::cout << "Selected cluster angle (rad, deg): " << angle << " " << angle*180.0/3.14159 << std::endl;
    std::cout << "Selected cluster hue: " << feats[selected_cluster_index].hue << std::endl;

    //Send it via ircp
    //send the object features to c6 here
    //fillObjectInfo(outFeatures);
    fillObjectInfo(feats[selected_cluster_index]);
    myIrcp.sendObjectInformation();

    //BELOW CODE IS HORRIBLE DO NOT USE ELSEWHERE!!! YOU'VE BEEN WARNED!

    //Get keypress, later ircp commands
    //getKeyPress(myIrcp.globalRequests); //TODO: implement it if need be

    commands4Loop c4l;
    c4l = myIrcp.handleCommands(baseName);

    if(ircpHandler::globalSkillInfo.hueValue >= 0)
        pA.hue_val = ircpHandler::globalSkillInfo.hueValue;

    if(c4l.loop_break)
            break;

    if(c4l.loop_continue)
            continue;

    if(c4l.make_skill_folder)
    {
      ircpHandler::globalSkillInfo.getSkillFolder(charBuff);
      makeFolder(charBuff);
    }

    if(c4l.save_raw)
    {
        /*if(saveIM)
        {
           // ircpHandler::globalSkillInfo.getFileNameWithExt(imageName,"png");
            sprintf(imageName,"%s.png",baseName);
            imwrite(imageName,inImageScaled);
        }*/
        //if(savePC)
        //{
          if(ircpHandler::globalSkillInfo.firstInfo)
          {
            ircpHandler::globalSkillInfo.getSkillFolder(charBuff);
            makeFolder(charBuff);
          }
          imageName_mutex.lock();
          //ircpHandler::globalSkillInfo.getFileNameWithExt(imageName,"pcd");
          sprintf(imageName,"%s.pcd",baseName);
          imageName_mutex.unlock();
          writePCD2File = true;
        //}
    }
  }

  //interface->stop ();
  return 0;
}