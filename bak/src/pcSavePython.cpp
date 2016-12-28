#include <common.hpp>
#include <pcl/conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>

#include <stdlib.h>
#include <signal.h>
#include <pthread.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>

pcl::PointCloud<PointT>::ConstPtr prev_cloud;
boost::mutex cloud_mutex;
boost::mutex imageName_mutex;
bool writePCD2File = false;
char imageName[100];
bool gotFirst = false;

//./def_loop -ur 1 -rt asus_filtered -dt 0.03 -ct 15 -t 14 -e 0.1 -v 40 -b 0
//./def_loop -ur 1 -rt asus_filtered -dt 0.03 -ct 15 -t 14 -e 0.1 -v 40 -b 0 -sh 0

//As always with my code, and the philosophy of c/c++, i won't pay for what I don't need:
//No bound checking on the arrays. Make sure you initialize it right. Make sure you use it right!!!
//Not that this code is intended for anybody else but if you end up using it, I will not
//help you fix the seg-faults. A good programmer writes <insert-favorite-language> in any language
//This involves bound checking if you need it!!!
//Note that this code is intended to be wrapped up with swig and the main goal is
//simplicity on the other end and efficiency on this end
//Why this rant? I had problems with noobs modifying my code and causing issues for all of us, more than twice...

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
      cloud_mutex.unlock ();
      std::cout << imageName << std::endl;
      imageName_mutex.lock();
      pcl::io::savePCDFile(imageName, *saved_cloud);
      writePCD2File = false;
      imageName_mutex.unlock();
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
      cloud_mutex.unlock ();
      std::cout << imageName << std::endl;
      imageName_mutex.lock();
      pcl::io::savePCDFile(imageName, *saved_cloud);
      writePCD2File = false;
      imageName_mutex.unlock();
    }
    else
      cloud_mutex.unlock ();

    gotFirst = true;
}

pcl::Grabber* interface;
ros::NodeHandle *nh;
ros::Subscriber sub;
bool use_ros;

unsigned int sleep_milisec = 1;

pthread_t spinForCallbacks;
bool quit = false;

void update()
{
  if(use_ros)
    ros::spinOnce();

  usleep(sleep_milisec);
}

void *spin(void *sleep_milisec)
{
  while(!quit)
    update();

  return 0;
}

void
init(bool _use_ros = true, char *ros_topic = "/asus/depth_registered/points", unsigned int _sleep_milisec = 10)
{
  use_ros = _use_ros;
  sleep_milisec = _sleep_milisec;
  int argc = 1;
  char **argv;
  argv = (char**) malloc(1*sizeof(char*));
  argv[0] = (char*) malloc(100*sizeof(char));
  sprintf(argv[0],"pcSavePython");
  if(use_ros)
  {
      std::cout << "Using ros" << std::endl;
      ros::init(argc, argv, "pc_segmentation",ros::init_options::NoSigintHandler);
      nh = new ros::NodeHandle();
      sub = nh->subscribe<sensor_msgs::PointCloud2>(ros_topic, 1, cloud_cb_ros_);
  }
  else
  {
      std::cout << "Using the device" << std::endl;
      interface = new pcl::OpenNIGrabber ();
      boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&cloud_cb_direct_,_1);
      boost::signals2::connection c = interface->registerCallback (f);
  }

  if(pthread_create(&spinForCallbacks, NULL, spin, (void *)sleep_milisec))
  {
    fprintf(stderr, "Error creating thread\n");
  }
}

void
save(const char *baseName)
{
  imageName_mutex.lock();
  sprintf(imageName,"%s.pcd",baseName);
  writePCD2File = true;
  imageName_mutex.unlock();
  return;
}


void
fini()
{
  quit = true;
  if(!use_ros)
  {
    interface->stop ();
    delete interface;
  }
  else
  {
    delete nh;
    ros::shutdown();
  }
}

//hopefully the saving function is serial :)
bool isSaving()
{
  return writePCD2File;
}


const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

//The feature stuff starts
#include<utils.hpp>
#include<pc_segmentation.hpp>
#include<utils_pcl_ros.hpp>

OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;
parsedArguments pA;

bool pA_initialized = false;
bool segmenter_initialized = false;

/*int charArray2charDoubleArray(char *in_char, char *** out_char, const char * delimiters)
{
  char * pch;

  char *tmp_char = (char*) malloc((strlen(in_char)+1)*sizeof(char));
  memcpy(tmp_char, in_char, strlen(in_char)+1);

  int arg_count = 0;
  pch = strtok (tmp_char,delimiters);
  while (pch != NULL)
  {
    pch = strtok (NULL, delimiters);
    arg_count++;
  }
  printf ("Number of arguments: %d\n",arg_count);

  free(tmp_char);

  *out_char = (char**) malloc((arg_count+2)*sizeof(char*));

  pch = strtok (in_char,delimiters);
  int i = 0;
  sprintf(*out_char[i], "pythonPcWrapper");
  i++;
  while (pch != NULL)
  {
    int char_len = strlen(pch);
    *out_char[i] = (char*) malloc((char_len+1)*sizeof(char)); //+1 is a lucky charm, don't know if i need it :)
    sprintf(*out_char[i],"%s",pch);
    printf ("(%s) ",*out_char[i]);
    pch = strtok (NULL, delimiters);
    i++;
  }
  printf ("\n");

  sprintf(*out_char[i],"bogus");

  if(i != arg_count + 1)
    std::cout << "Problem..." << std::endl;
  return i;
}*/

void initPA(char *_argv)
{
  char **argv;
  char *delimiters = " ";

  std::cout << "Parsing arguments" << std::endl;

  //begin: UGLY CODE for string stuff, ignore
  char * pch;

  char *tmp_char = (char*) malloc((strlen(_argv)+1)*sizeof(char));
  memcpy(tmp_char, _argv, strlen(_argv)+1);

  int arg_count = 0;
  pch = strtok (tmp_char,delimiters);
  while (pch != NULL)
  {
    pch = strtok (NULL, delimiters);
    arg_count++;
  }
  free(tmp_char);
  printf ("Number of arguments: %d\n",arg_count);

  argv = (char**) malloc((arg_count+2)*sizeof(char*));
  pch = strtok (_argv,delimiters);
  int i = 0;
  argv[i] = (char*) malloc((16)*sizeof(char)); //+1 is a lucky charm, don't know if i need it :)
  sprintf(argv[i], "pythonPcWrapper");
  i++;
  while (pch != NULL)
  {
    int char_len = strlen(pch);
    argv[i] = (char*) malloc((char_len+1)*sizeof(char)); //+1 is a lucky charm
    sprintf(argv[i],"%s",pch);
    printf ("(%s) ",argv[i]);
    pch = strtok (NULL, delimiters);
    i++;
  }
  printf ("\n");
  //std::cout << "Done parsing arguments" << std::endl;

  //sprintf(argv[i],"bogus");
  //std::cout << "Done parsing arguments" << std::endl;

  if(i != arg_count + 1)
    printf("Problem...\n");
  else
    arg_count++;

  //end: UGLY CODE


  parseArguments(arg_count, argv, pA);
  pA_initialized = true;

  for (int i=0; i<arg_count; i++)
    free(argv[i]);

  std::cout << "Done parsing arguments" << std::endl;
}

//one day i will add the workspace to parsedArguments, it is trivial really but why don't I just do it?
void initFeats(char *_argv = NULL)
{
  if(_argv != NULL)
    initPA(_argv);

  std::cout << "Initializing the segmenter" << std::endl;
  if(!pA_initialized)
    std::cout << "Arguments not initialized. Using default values" << std::endl;

  float workSpace[] = {-0.6,0.6,-0.5,0.5,0.4,1.1};//{-0.5,0.6,-0.4,0.4,0.4,1.1};
  multi_plane_app.setWorkingVolumeThresholds(workSpace);
  multi_plane_app.initSegmentation(pA.seg_color_ind, pA.ecc_dist_thresh, pA.ecc_color_thresh);
  multi_plane_app.verbose = false;
  segmenter_initialized = true;
}

void getFeatures(pcl::PointCloud<PointT>::ConstPtr cloud, float *out_feats)
{
  if(!segmenter_initialized)
  {
    std::cout << "Segmenter not initialized. Initializing for you... You're welcome." << std::endl;
    initFeats();
  }
  std::vector<pc_cluster_features> feats;
  int
  selected_cluster_index = multi_plane_app.run3(cloud,feats,pA.hue_val,pA.hue_thresh, pA.z_thresh, pA.euc_thresh, pA.pre_proc, pA.seg_color_ind, pA.merge_clusters, pA.displayAllBb, false);
  cloud_mutex.unlock();
  feats[selected_cluster_index].fillFeatureContainer(out_feats,0);
}

bool getCurrentFeatures(float *out_feats, bool wait_for_availability = true, bool save_pc = false)
{
  bool ret_val = false;

  do
  {
    if(cloud_mutex.try_lock ())
    {
      getFeatures(prev_cloud, out_feats);
      ret_val = true;
      if(save_pc)
      {
        std::string fileName = "demo_" + currentDateTime() + ".psd";
        save(fileName.c_str());
      }

      break;
    }
  } while(wait_for_availability);

  return ret_val;

}

bool getFileFeatures(float *out_feats, char *file_name)
{
  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  if(!loadPCD(file_name,cloud_ptr))
  {
    return false;
  }
  getFeatures(cloud_ptr, out_feats);
  return true;
}

int getNumberOfFeatures()
{
  return 334;
}


