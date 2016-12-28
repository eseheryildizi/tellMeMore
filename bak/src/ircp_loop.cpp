/*
 * ircp_loop.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: baris
 */

#include<ircpHandler.hpp>
#include<pc_segmentation.hpp>

pcl::PointCloud<PointT>::ConstPtr prev_cloud;
boost::mutex cloud_mutex;
boost::mutex imageName_mutex;
bool writePCD2File = false;
char imageName[100];

void
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
}

int
main (int argc, char **argv)
{

  parsedArguments pA;
  if(parseArguments(argc, argv, pA) < 0)
    return 0;

  /*parsedArguments2 pA2;
  if(parseArguments2(argc, argv, pA2) < 0)
    return 0;

  //it is easy to add new arguments by using the value2parse class and pA2.values.push_back(...);
  //however, you have to now how to use the value later*/

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

  pcl::Grabber* interface = new pcl::OpenNIGrabber ();

  boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&cloud_cb_,_1);
  boost::signals2::connection c = interface->registerCallback (f);

  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = cloudViewer(cloud_ptr);

  multi_plane_app.initSegmentation(pA.seg_color_ind, pA.ecc_dist_thresh, pA.ecc_color_thresh);
  multi_plane_app.setViewer(viewer);

  resetServiceRequest(ircpHandler::globalRequests);

  float selected_object_features[324];

  double workSpace[] = {-0.55,0.6,-0.4,0.4,0.4,1.1};
  multi_plane_app.setWorkingVolumeThresholds(workSpace);


  interface->start ();
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce(50);
    std::vector<pc_cluster_features> feats;
    int selected_cluster_index = -1;
    if(cloud_mutex.try_lock ())
    {
      selected_cluster_index = multi_plane_app.run3(prev_cloud,feats,pA.hue_val,pA.hue_thresh, pA.z_thresh, pA.euc_thresh, pA.pre_proc, pA.seg_color_ind, pA.merge_clusters);
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

  interface->stop ();
  return 0;
}
