/*
 * ircp_loop.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: baris
 */

#include<pc_segmentation.hpp>
#include<ircpHandler.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

struct parsedArguments
{
  double hue_val;
  double hue_thresh;
  double z_thresh;

  parsedArguments() : hue_val(230.0), hue_thresh(5.0), z_thresh(0.07){}
};

int
parseArguments(int argc, char **argv, parsedArguments &pA)
{
  int unknownArgNum = 0;
  for (int i = 1; i < argc; i++)
  {
      if (!strcmp(argv[i],"-v"))
      {
          pA.hue_val = atoi(argv[++i]);
      }
      else if (!strcmp(argv[i],"-t"))
      {
          pA.hue_thresh = atoi(argv[++i]);
      }
      else if (!strcmp(argv[i],"-z"))
      {
          pA.z_thresh = atoi(argv[++i]);
      }
      else if (!strcmp(argv[i],"-h"))
      {
          std::cout << "-v: hue_val, -t: hue_tresh, -z: z_thresh, -h: this help" << std::endl;
          return -1;
      }
      else
      {
          cout << "Unknown argument: " << argv[i] << endl;
          unknownArgNum++;
      }
  }
  return unknownArgNum;
}

int
main (int argc, char **argv)
{
  parsedArguments pA;

  if(parseArguments(argc, argv, pA) < 0)
    return 0;

  Pylon::PylonAutoInitTerm autoInitTerm;

  BaslerCamera baslerCam;
  BaslerCamera::options parsedOptions = parseArguments(argc, argv);

  baslerCam.initialize(parsedOptions);

  double scale = parsedOptions.scale;
  Size imageSize = baslerCam.getCvSize(scale);
  uint imageWidth = baslerCam.getWidth(scale);
  uint imageHeight = baslerCam.getHeight(scale);
  bool isColor = baslerCam.isColor();
  int imageType = CV_8UC3; //if is color
  uint fpsWrite = imageHeight*0.95;

  Mat inImage = baslerCam.createCvImage(true);
  Mat inImageScaled = baslerCam.createCvImage(true, scale);

  namedWindow("Image",  CV_WINDOW_AUTOSIZE); moveWindow("Image", 100, 100);

  //IRCP RELATED
  unsigned char module = OVERHEAD_VISION_BASLER_MODULE_ID;
  unsigned char targetModule = SIMON_BEHAVIOR_MODULE_ID;
  unsigned char robot = SIMON_ID;

  ircpHandler myIrcp(targetModule, module, robot);
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

  multi_plane_app.initSegmentation();
  multi_plane_app.setViewer(viewer);

  resetServiceRequest(ircpHandler::globalRequests);

  float selected_object_features[324];
  timingHelper fpsGetter;
  fpsGetter.init();

  baslerCam.startGrab();

  interface->start ();
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce(50);
    std::vector<pc_cluster_features> feats;
    int selected_cluster_index = -1;
    if(cloud_mutex.try_lock ())
    {
      selected_cluster_index = multi_plane_app.run3(prev_cloud,feats,pA.hue_val,pA.hue_thresh, pA.z_thresh);
      cloud_mutex.unlock();

      /*the z_thresh may result in wrong cluster to be selected. it might be a good idea to do
      * some sort of mean shift tracking, or selecting the biggest amongst the candidates (vs choosing the most similar color)
      * or sending all the plausible ones to c6 and decide there
      */
    }

    baslerCam.getImage(inImage);
    resize(inImage, inImageScaled, imageSize);

    waitKey(100);
    if(isViz)
    {
            //Display stuff
            fpsGetter.check();
            sprintf(tmpS,"fps: %f",fpsGetter.getFPS());


            inImageScaled.copyTo(inImageScaledForViz);
            writeText(inImageScaledForViz,tmpS,Point(5,fpsWrite));
            putGridLinesInImage(inImageScaledForViz, 4, 6);
            imshow("Image",inImageScaledForViz);

    }

    if(selected_cluster_index < 0)
      continue;

    float angle = feats[selected_cluster_index].aligned_bounding_box.angle;
    std::cout << "Selected clsuter angle (rad, deg): " << angle << " " << angle*180.0/3.14159 << std::endl;
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