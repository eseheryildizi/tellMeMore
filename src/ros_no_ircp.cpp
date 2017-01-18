/*
 * ros_no_ircp.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: baris
 */
//#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

#include <pc_segmentation.hpp>
#include <objectFeatures.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/openni_grabber.h>

#include <utils.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_conversions2.hpp>

#include <k2g.h>
#include <serviceHelper.hpp>
#include <signal.h>
#include <curses.h>

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <stdio.h>

pcl::PointCloud<PointT>::ConstPtr prev_cloud;
boost::mutex cloud_mutex;
boost::mutex imageName_mutex;
bool writePCD2File = false;
char imageName[100];
bool gotFirst = false;
int keypressed;


using namespace std;

void
cloud_cb_ros_ (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PCLPointCloud2 pcl_pc;

  //might not work for all the sensors!!!
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

inline void
fake_cloud_cb_kinectv2_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
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
}
int stringToInt(string s, std::vector<string> v){
    auto it = std::find(v.begin(), v.end(), s);
    int i = -1;
    if( it != v.end()) {
    i = it - v.begin();
  } 
  return i;
}
int 
writeData(std::vector<pc_cluster_features> feats, bool ask)
{
  float featArray[335];
 /* std::vector<string> colors = {"red","green","blue","yellow","orange","white","black"};
  std::vector<string> sizes = {"small","medium","big"};
  std::vector<string> shapes = {"cube","sphere","cylindre"};

  ofstream outputFile;
  std::string col, fname, size, shape;
  int colInt,sizeInt,shapeInt;
  cout<<"What color is it?";
  cin>>col;
  while(stringToInt(col,colors)==-1){
  cout<<"No such color! ";
  cout<<"What color is it?";
  cin>>col;
  }
  colInt = stringToInt(col,colors);

  cout<<"What is its shape?";
  cin>>shape;
  while(stringToInt(shape,shapes)==-1){
  cout<<"No such shape! ";
  cout<<"What is its shape?";
  cin>>shape;
  }
  shapeInt = stringToInt(shape,shapes);

  cout<<"How is the size? Big? Medium? Small?";
  cin>>size;
  while(stringToInt(size,sizes)==-1){
  cout<<"No such size! ";
  cout<<"How is the size? Big? Medium? Small?";
  cin>>size;
  }
  sizeInt = stringToInt(size,sizes);

  fname = size+col+shape+".txt";
  outputFile.open(fname);
  */
  std::string f = "";
  for(int x = 0; x < feats.size(); x++){
    if(x!=0)
      f.append("\n");
    feats.at(x).fillFeatureContainer(featArray,0);
    for(int i = 0; i < sizeof(featArray)/sizeof(*featArray); i++){
      if(i != 0)
       f.append(" ");
       f.append(std::to_string(featArray[i]));
      }
  }
  std::string cmd;
  if(!ask)
  cmd = "gnome-terminal -x sh -c './getData.sh "+f+ "; exec bash'"; 
  if(ask)
  cmd = "gnome-terminal -x sh -c './ask.sh "+f+ "; exec bash'";   
  
  return system(cmd.c_str ());
/*  outputFile<<", ";
  outputFile<<colInt;
  outputFile<<", ";
  outputFile<<shapeInt;
  outputFile<<", ";
  outputFile<<sizeInt;
  outputFile << "}" << endl;
  outputFile.close();
  */

}

int
main (int argc, char **argv)
{
 /*     const char *dev = "/dev/input/event3";
    struct input_event ev;
    ssize_t n;
    int fd;
    */
  parsedArguments pA;
  if(parseArguments(argc, argv, pA) < 0)
    return 0;

  ridiculous_global_variables::ignore_low_sat       = pA.saturation_hack;
  ridiculous_global_variables::saturation_threshold = pA.saturation_hack_value;
  ridiculous_global_variables::saturation_mapped_value = pA.saturation_mapped_value;

  //  ircpHandler myIrcp(BEHAVIOR_MODULE_ID, module, robot);
  //  myIrcp.initialize();

  globalSkillInfo.dataLocation = "/home/baris/data/newData";//"/home/baris/data/taylor-exp";//"/home/baris/data/iros15";//agLearning";

  //Helpers

  char charBuff[100];
  char baseName[200];
  char rogueSKillCounter = 48; //0 in ascii

  std::stringstream skillName;
  skillName << "skill" << rogueSKillCounter;
  globalSkillInfo.skillName = skillName.str();

  //Demo information
  std::vector<int> numberOfKeyframes;

  makeFolder(globalSkillInfo.dataLocation.c_str());
  globalSkillInfo.getSubjectFolder(charBuff);
  makeFolder(charBuff);

  char tmpS[100];
  bool idleMode = true;

  resetServiceRequest(globalRequests);

  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;

  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = cloudViewer(cloud_ptr);

  multi_plane_app.initSegmentation(pA.seg_color_ind, pA.ecc_dist_thresh, pA.ecc_color_thresh);
  multi_plane_app.setViewer(viewer);

  float selected_object_features[324];

  float workSpace[] = {-0.3,0.4,-0.25,0.35,0.3,2.0};//Simon on the other side:{-0.1,0.6,-0.4,0.15,0.7,1.1};//{-0.5,0.6,-0.4,0.4,0.4,1.1};
  multi_plane_app.setWorkingVolumeThresholds(workSpace);

  pcl::Grabber* interface;
  ros::NodeHandle *nh;
  ros::Subscriber sub;
  K2G *k2g;
  processor freenectprocessor = OPENGL;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

  switch (pA.pc_source)
  {
    case 0:
    {
      std::cout << "Using ros topic as input" << std::endl;
      ros::init(argc, argv, "pc_segmentation",ros::init_options::NoSigintHandler);
      nh = new ros::NodeHandle();
      sub = nh->subscribe<sensor_msgs::PointCloud2>(pA.ros_topic, 1, cloud_cb_ros_);
      break;
    }
    case 1:
    {
      std::cout << "Using the openni device" << std::endl;

      interface = new pcl::OpenNIGrabber ();

      boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&cloud_cb_direct_,_1);
      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();
      break;
    }
    case 2:
    default:
    {
      std::cout << "Using kinect v2" << std::endl;

      freenectprocessor = static_cast<processor>(pA.freenectProcessor);

      k2g = new K2G(freenectprocessor, true);
      cloud = k2g->getCloud();
      prev_cloud = cloud;
      gotFirst = true;
      break;
    }
  }

  /*ofstream myfile;
  myfile.open ("centroids.txt");*/
  

  //int isFileWritten = 0;


 //burayı durdur!!!
  while (!viewer->wasStopped ())
  {
    keypressed=nonBlockingWait(0,1);

    if(pA.ros_node)
    {
      ros::spinOnce();
    }
    viewer->spinOnce(20);
    if(!gotFirst)
      continue;
    std::vector<pc_cluster_features> feats;
    int selected_cluster_index = -1;
    if(cloud_mutex.try_lock())
    {
      if(pA.pc_source == 2)
      {
        cloud = k2g->getCloud();
        fake_cloud_cb_kinectv2_(cloud);
      }

      selected_cluster_index = multi_plane_app.processOnce(prev_cloud,feats,pA.hue_val,pA.hue_thresh, pA.z_thresh, pA.euc_thresh, pA.pre_proc, pA.seg_color_ind, pA.merge_clusters, pA.displayAllBb);
      cloud_mutex.unlock();

      /*the z_thresh may result in wrong cluster to be selected. it might be a good idea to do
       * some sort of mean shift tracking, or selecting the biggest amongst the candidates (vs choosing the most similar color)
       * or sending all the plausible ones to c6 and decide there
       */
    }

    if(selected_cluster_index < 0)
      continue;

    float angle = feats[selected_cluster_index].aligned_bounding_box.angle;
    std::cout << "Selected cluster angle (rad, deg): " << angle << " " << angle*180.0/3.14159 << std::endl;
    std::cout << "Selected cluster hue: " << feats[selected_cluster_index].hue << std::endl;

    //Send it via ircp
    //send the object features to c6 here
    //fillObjectInfo(outFeatures);
    fillObjectInfo(feats[selected_cluster_index]);
    //    myIrcp.sendObjectInformation();

    /*if(!isFileWritten && selected_cluster_index>=0){
        gettimeofday(&tv, 0);
        gettimeofday(&tv2, 0);
        while(tv2-tv <3000)
          gettimeofday(&tv2,0);*/


    commands4Loop c4l;
    //    c4l = myIrcp.handleCommands(baseName);

    if(globalSkillInfo.hueValue >= 0)
      pA.hue_val = globalSkillInfo.hueValue;

    if(c4l.loop_break)
      break;

    if(c4l.loop_continue)
      continue;

    if(c4l.make_skill_folder)
    {
      globalSkillInfo.getSkillFolder(charBuff);
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
      if(globalSkillInfo.firstInfo)
      {
        globalSkillInfo.getSkillFolder(charBuff);
        makeFolder(charBuff);
      }
      imageName_mutex.lock();
      //globalSkillInfo.getFileNameWithExt(imageName,"pcd");
      sprintf(imageName,"%s.pcd",baseName);
      imageName_mutex.unlock();
      writePCD2File = true;
      //}

    }
        if(keypressed == 'p')
         writeData(feats,false);
        if(keypressed == 'a')
         writeData(feats,true);
    
 /*   if (cin.get() == '\n'){
        gData = true;
    //  isFileWritten = 1;
    }*/
    //  ros::shutdown();


/*    fd = open(dev, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Cannot open %s: %s.\n", dev, strerror(errno));
        return EXIT_FAILURE;
    }
        while (1) {
        n = read(fd, &ev, sizeof ev);
        if (n == (ssize_t)-1) {
            if (errno == EINTR)
                continue;
            else
                break;
        } else
        if (n != sizeof ev) {
            errno = EIO;
            break;
        }
                if (ev.type == EV_KEY && ev.value >= 0 && ev.value <= 2)
                    writeData(feats);

    }
    fflush(stdout);
    fprintf(stderr, "%s.\n", strerror(errno));
    */
  }
  if(pA.pc_source == 1)
    {
      interface->stop ();
      delete interface;
    }
    if(pA.ros_node)
    {
      delete nh;

    //myfile.close();
      ros::shutdown();
    }

    return 0;
}
