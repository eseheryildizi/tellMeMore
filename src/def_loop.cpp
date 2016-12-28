/*
 * ircp_loop.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: baris
 */

#include <ircpHandler.hpp>
#include <pc_segmentation.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/openni_grabber.h>

#include <utils.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utils_pcl_ros.hpp>
#include <geometry_msgs/Transform.h>

#include <k2g.h>

#include <signal.h>

pcl::PointCloud<PointT>::ConstPtr prev_cloud;
boost::mutex cloud_mutex;
boost::mutex imageName_mutex;
bool writePCD2File = false;
char imageName[100];
bool gotFirst = false;
bool interrupt = false;

//./def_loop -src 1 -rt asus_filtered -dt 0.03 -ct 15 -t 14 -e 0.1 -v 40 -b 0
//./def_loop -src 1 -rt asus_filtered -dt 0.03 -ct 15 -t 14 -e 0.1 -v 40 -b 0 -sh 0
//./def_loop -src 1 -rt asus_filtered -dt 0.03 -ct 15 -t 14 -e 0.1 -v 200 -sh 1 -st 0.2 -sv 200
//./def_loop -dt 0.03 -ct 15 -t 14 -e 0.1 -v 340 -sh 1 -st 0.2 -sv 240
//./def_loop -dt 0.03 -ct 15 -t 14 -e 0.1 -v 221 -sh 1 -st 0.4 -sv 140


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

void interruptFn(int sig) 
{
  interrupt = true;
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

int
main (int argc, char **argv)
{

  parsedArguments pA;
  if(parseArguments(argc, argv, pA) < 0)
    return 0;

  ridiculous_global_variables::ignore_low_sat       = pA.saturation_hack;
  ridiculous_global_variables::saturation_threshold = pA.saturation_hack_value;
  ridiculous_global_variables::saturation_mapped_value = pA.saturation_mapped_value;

  /*parsedArguments2 pA2;
  if(parseArguments2(argc, argv, pA2) < 0)
    return 0;

  //it is easy to add new arguments by using the value2parse class and pA2.values.push_back(...);
  //however, you have to now how to use the value later*/

  //IRCP RELATED
  unsigned char module = OVERHEAD_VISION_BASLER_MODULE_ID;
  unsigned char targetModule = BEHAVIOR_MODULE_ID;
  unsigned char robot = pA.robot_id;//SIMON_ID;

  bool registerCallbacks = (pA.comm_medium == comType::cIRCP);
  bool enableIrcp = (registerCallbacks || pA.output_type == cIRCP);

  ircpHandler *myIrcp;

  if(enableIrcp)
  {
    myIrcp = new ircpHandler(BEHAVIOR_MODULE_ID, module, robot, registerCallbacks);
    myIrcp->initialize();
  }

  ircpHandler::globalSkillInfo.dataLocation = "/home/baris/data/agLearningNaive";//"/home/baris/data/taylor-exp";//"/home/baris/data/iros15";//agLearning";

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

  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;

  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  multi_plane_app.initSegmentation(pA.seg_color_ind, pA.ecc_dist_thresh, pA.ecc_color_thresh);
  if(pA.viz) 
  {
    viewer = cloudViewer(cloud_ptr);
    multi_plane_app.setViewer(viewer);
  }

  resetServiceRequest(ircpHandler::globalRequests);

  float selected_object_features[324];

  float workSpace[] = {-0.6,0.6,-0.5,0.5,0.3,2.0};//Simon on the other side:{-0.1,0.6,-0.4,0.15,0.7,1.1};//{-0.5,0.6,-0.4,0.4,0.4,1.1};
  multi_plane_app.setWorkingVolumeThresholds(workSpace);

  pcl::Grabber* interface;
  ros::NodeHandle *nh;
  ros::Subscriber sub;
  ros::Publisher pub;

  ros::Publisher transformPub;

  bool spawnObject = true;
  ros::Publisher objMarkerPub;

  K2G *k2g;
  processor freenectprocessor = OPENGL;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

  if(pA.ros_node)
  {
    std::cout << "ros node initialized" << std::endl;
    ros::init(argc, argv, "pc_segmentation",ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle();
  }

  const char *outRostopic = "/baris/features";
  const char *outRostopicTransform = "/baris/objectTransform";

  if(pA.output_type == comType::cROS)
  {
    std::cout << "Publishing ros topic: " << outRostopic << std::endl;
    pub = nh->advertise<pc_segmentation::PcFeatures>(outRostopic,5);

    transformPub = nh->advertise<geometry_msgs::Transform>(outRostopicTransform,5);

    if(spawnObject)
    {
      objMarkerPub = nh->advertise<visualization_msgs::Marker>("/baris/object_marker", 1);
    }
  }

  switch (pA.pc_source)
  {
    case 0:
    {
      std::cout << "Using ros topic as input" << std::endl;
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
  if(!pA.viz)
   signal(SIGINT, interruptFn);
  while (!interrupt && (!pA.viz || !viewer->wasStopped ()))
  {
    if(pA.ros_node)
    {
      ros::spinOnce();
    }
    if(pA.viz)
      viewer->spinOnce(20);
    if(!gotFirst)
      continue;
    std::vector<pc_cluster_features> feats;
    int selected_cluster_index = -1;
    if(cloud_mutex.try_lock ())
    {
      if(pA.pc_source == 2)
      {
        cloud = k2g->getCloud();
        fake_cloud_cb_kinectv2_(cloud);
      }

  	  if(pA.viz && pA.justViewPointCloud)
  	  {
  		pcl::PointCloud<PointT>::Ptr filtered_prev_cloud(new pcl::PointCloud<PointT>(*prev_cloud));
  		multi_plane_app.preProcPointCloud(filtered_prev_cloud);
  		if (!viewer->updatePointCloud<PointT> (filtered_prev_cloud, "cloud"))
          {
              viewer->addPointCloud<PointT> (filtered_prev_cloud, "cloud");
          }
          selected_cluster_index = -1;
  	  }
  	  else
  	  {
  		  selected_cluster_index = multi_plane_app.processOnce(prev_cloud,feats,pA.hue_val,pA.hue_thresh, pA.z_thresh, pA.euc_thresh, pA.pre_proc,
  				  	  	  	  	  	  	  	  	  	  	  	   pA.seg_color_ind, pA.merge_clusters, pA.displayAllBb, pA.viz, pA.filterNoise); //true is for the viewer

  	  }

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

    switch (pA.output_type)
    {
      case 1: //ircp
      {
        myIrcp->sendObjectInformation();
        fillObjectInfo(feats[selected_cluster_index]);
        break;
      }
      case 2:
      default:
      {
        pc_segmentation::PcFeatures rosMsg;
        fillRosMessage(rosMsg, feats[selected_cluster_index]);
        pub.publish(rosMsg);
        transformPub.publish(rosMsg.transform);
        objectPoseTF(rosMsg.transform);
        if(spawnObject)
        {
          visualization_msgs::Marker marker;
          getObjectMarker(marker, rosMsg);
          objMarkerPub.publish(marker);
        }
        break;
      }
    }

    /*myfile  <<        feats[selected_cluster_index].centroid[0]
            << " " << feats[selected_cluster_index].centroid[1]
            << " " << feats[selected_cluster_index].centroid[2]
            << " " << feats[selected_cluster_index].aligned_bounding_box.angle
            << " " << feats[selected_cluster_index].aligned_bounding_box.center.x
		    << " " << feats[selected_cluster_index].aligned_bounding_box.center.y
		    << " " << feats[selected_cluster_index].aligned_bounding_box.center.z 
		    << std::endl;*/

    //BELOW CODE IS HORRIBLE DO NOT USE ELSEWHERE!!! YOU'VE BEEN WARNED!

    //Get keypress, later ircp commands
    //getKeyPress(myIrcp.globalRequests); //TODO: implement it if need be

    commands4Loop c4l;
    if(enableIrcp)
      c4l = myIrcp->handleCommands(baseName);
    //else
    //  c4l frpm ros

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
  if(enableIrcp)
  {
    delete myIrcp;
  }
  return 0;
}
