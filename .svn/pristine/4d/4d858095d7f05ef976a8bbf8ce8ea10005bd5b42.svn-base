#include<pc_segmentation.hpp>
#include<utils_pcl_ros.hpp>
#include <utils.hpp>

int starting_subject = 0;
int starting_demo = 0;
int starting_kf = 0;
int starting_skill = 0;
bool is_set_z_thresh = false;
bool is_viz_only = false;
bool batch = false;

void readInputArgs(int argc, char *argv[])
{
  for(int i =1; i < argc; i++)
  {
    if(!strcmp(argv[i],"-on"))
    {
      starting_subject = atoi(argv[++i]);
    }
    else if(!strcmp(argv[i],"-os"))
    {
      starting_skill = atoi(argv[++i]);
    }
    else if(!strcmp(argv[i],"-od"))
    {
      starting_demo = atoi(argv[++i]);
    }
    else if(!strcmp(argv[i],"-ok"))
    {
      starting_kf = atoi(argv[++i]);
    }
    else if(!strcmp(argv[i],"-ov"))
    {
      i++;
      is_viz_only = true;
    }
    else if(!strcmp(argv[i],"-batch"))
    {
        i++;
        batch = true;
    }
  }
}

//./viewAndProcess -dt 0.03 -ct 15 -t 14 -e 0.1 -v 221 -sh 1 -st 0.4 -sv 140

int main(int argc, char *argv[])
{
  readInputArgs(argc, argv);
  parsedArguments pA;
  parseArguments(argc,argv,pA);

  ridiculous_global_variables::ignore_low_sat       = pA.saturation_hack;
  ridiculous_global_variables::saturation_threshold = pA.saturation_hack_value;
  ridiculous_global_variables::saturation_mapped_value = pA.saturation_mapped_value;

  bool only_color = false;
  bool no_color = !only_color && true;

  OpenNIOrganizedMultiPlaneSegmentation segmenter;
  char folder_path[] = "/home/baris/data/agLearningNaive";
  char subject_base[] = "subject";
  char folder_base[] = "pc";
  char *skill_names[] = {"close the box", "open the box", "pour"};//{"do something", "close the box","pour"};//{"test_close the box","test_pour"};//{"close the box", "insert", "place"};
  char demo_base[] = "demo";
  char kf_base[] = "kf";
  char separator[] = "_";
  char ext[] = "pcd";
  char processed_folder[] = "proc";
  char name[1024];

  /*char folder_path[] = "./";
  char skill_name[] = "iter";
  char object_name[] = "cluster";
  char separator[] = "_";
  char name[1024];*/

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  viewer = cloudViewer(cloud_ptr);

  int keyPress;
  bool jumpOut = false;
  bool give_info = false;
  bool load_pc = true;
  bool process_pc = false;
  bool viz_proc_result = false;
  bool viz_ind_clusters = true;
  bool save_features = false;
  bool isObs = false;
  bool isCls = false;
  bool is_select_clusters = false;
  bool save_selected_cluster = false;
  bool save_euclidean_indices = false;
  bool massive_save = false;
  bool save_separate = false;
  bool separate_white = false;
  bool auto_merge = true;
  int selectedCluster = 0;
  int subject_num = starting_subject;//why did i have -1? -1;//0;
  int skill_num = starting_skill;
  int demo_num = starting_demo;
  int kf_num = starting_kf;

  bool has_subjects = strcmp(subject_base,"");

  int cluster_num = 0;

  std::vector<int> selected_clusters;

  int subject_list[] = {12,13,14,15,16,17,18,19,20,21,22,23};//,9,10};

  segmenter.initSegmentation(pA.seg_color_ind, pA.ecc_dist_thresh, pA.ecc_color_thresh);
  segmenter.setViewer(viewer);
  float threshs[] = {-0.3,0.4,-0.25,0.35,0.7,1.1	};
  segmenter.setWorkingVolumeThresholds(threshs);

  size_t prev_models_size = 0;
  size_t prev_cluster_num = 0;
  int prev_boundingbox_num = 0;

  pcl::PointCloud<PointT>::CloudVectorType *used_clusters;
  std::vector<pcl::PointCloud<pcl::Normal> > *used_cluster_normals;
  std::vector<pc_cluster_features> feats;
  std::vector<Box3D> fittedBoxes;
  std::vector<ColorVec > cluster_colors;
  pcl::PointCloud<PointT>::CloudVectorType clusters;
  std::vector<pcl::PointCloud<pcl::Normal> > cluster_normals;
  pcl::PointCloud<PointT>::CloudVectorType color_clusters;
  std::vector<pcl::PointCloud<pcl::Normal> > color_cluster_normals;
  pcl::PointCloud<PointT>::CloudVectorType merged_clusters;
  std::vector<pcl::PointCloud<pcl::Normal> > merged_normals;

  //int merge_list[2];
  std::vector<int> merge_list;
  std::vector<std::vector<int> > merge_list_list;
  bool is_merge = false;
  pcl::PointCloud<PointT> merged_cluster;
  pcl::PointCloud<pcl::Normal> merged_cluster_normal;

  pcl::PointCloud<PointT> saved_cluster_xyzrgba;
  pcl::PointCloud<pcl::Normal> saved_cluster_normal;
  pcl::PointCloud<pcl::PointXYZRGBNormal> saved_cluster_all;

  std::vector<pcl::PointIndices> label_indices;

  std::stringstream base_skill_path;
  char bsp[1024];

  float pct = 0;
  float rct = 0;

  double color_thresh = pA.ecc_color_thresh;

  //from visualization
  segmenter.removeCoordinateSystem();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (50);

    //min bytes to read, min time to wait in 10^{-1} seconds,
    keyPress = nonBlockingWait(0,1);    
    if(keyPress != EOF)
    {
      std::cout << std::endl;
    }
    switch (keyPress)
    {
      case 27: //esc
        jumpOut = true;
        break;
      case 'n': //next subject
        subject_num++;
        demo_num = 0;
        kf_num = 0;
        load_pc = true;
        break;
      case 'b': //previous subject
        subject_num--; if(subject_num < 0) subject_num = 0;
        demo_num = 0;
        kf_num = 0;
        load_pc = true;
        break;
      case 'w':
        skill_num++;
        demo_num = 0;
        kf_num = 0;
        load_pc = true;
        break;
      case 's':
        skill_num--; if(skill_num < 0) skill_num = 0;
        kf_num = 0;
        demo_num = 0;
        load_pc = true;
        break;
      case 'a':
        demo_num--; if(demo_num < 0) demo_num = 0;
        kf_num = 0;
        load_pc = true;
        break;
      case 'd':
        demo_num++;
        kf_num = 0;
        load_pc = true;
        break;
      case 'k':
        kf_num++;
        load_pc = true;
        break;
      case 'j':
        kf_num--; if(kf_num < 0) kf_num = 0;
        load_pc = true;
        break;
      case 'i':
        give_info = true;
        break;
      case 'l':
        load_pc = true;
        break;
      case 'p':
        process_pc = true;
        break;
      case 'v':
        viz_proc_result = true;
        break;
      case 'r':
        subject_num = 0;
        skill_num = 0;
        demo_num = 0;
        kf_num = 0;
        merge_list.clear();
        merge_list_list.clear();
        selected_clusters.clear();
        break;
      case 'c':
        viz_ind_clusters = !viz_ind_clusters;
        break;
      case 'x':
        selectedCluster++;
        std::cout << "Current cluster: " << selectedCluster << std::endl;
        break;
      case 'z':
        selectedCluster--; if(selectedCluster < 0) selectedCluster = 0;
        std::cout << "Current cluster: " << selectedCluster << std::endl;
        break;
      case 'o':
        isObs = !isObs;
        std::cout << "Observation: " << isObs << std::endl;
        break;
      case 'y':
        isCls = !isCls;
        std::cout << "Using Clusters: " << isCls << std::endl; //using saved indices
        break;
      case 't':
      {
        is_select_clusters = !is_select_clusters;
        massive_save = is_select_clusters;
        save_separate = is_select_clusters;
        if(!is_select_clusters)
          break;
        bool flag = true;
        selected_clusters.clear();
        int tmp;
        while (flag)
        {
          std::cout << "Cluster " << selected_clusters.size()+1 << ": ";
          cin >> tmp;

          if(tmp < 0)
            flag = false;
          else
            selected_clusters.push_back(tmp);
        }
        std::cout << "Selected Clusters List: " << selected_clusters.size()  << std::endl;
        break;
      }
      case 'm':
      {
        is_merge = !is_merge;
        if(!is_merge)
          break;
        bool flag = true;
        bool new_list = false;
        merge_list.clear();
        merge_list_list.clear();
        int tmp;
        while(flag)
        {
          if(new_list)
          {
            merge_list.clear();
            new_list = false;
          }

          while (!new_list)
          {
            std::cout << "Cluster " << merge_list.size()+1 << ": ";
            cin >> tmp;

            if (tmp == -9)
            {
              new_list = true;
              break;
            }
            else if(tmp < 0)
            {
              flag = false;
              break;
            }
            else
              merge_list.push_back(tmp);
          }
          std::cout << "Merge List: " << merge_list.size()  << std::endl;
          merge_list_list.push_back(merge_list);
        }
        break;
      }
      case 'q':
        //no_color = !no_color;
        if(color_thresh == 10000)
          color_thresh = pA.ecc_color_thresh;
        else
          color_thresh = 10000;
        segmenter.euclidean_cluster_comparator_->setColorThreshold(color_thresh);
        break;
      case 'f':
        if(!viz_ind_clusters)
        {
          cout << "First press c to enter individual cluster mode." << endl;
          break;
        }
        //cout << "Saving features" << endl;
        save_features= true;
        break;
      case 'e': //automerge on
        auto_merge = !auto_merge;
        break;
      case 'g':
        /*if(!viz_ind_clusters)
        {
          cout << "First press c to enter individual cluster mode." << endl;
          break;
        }*/
        //cout << "Saving selected cluster" << endl;
        //save_selected_cluster = true;
        save_euclidean_indices = true;
        cluster_num++;
        break;
      case '+':
        pA.hue_val++;
        process_pc = true;
        std::cout << "New hue val: " << pA.hue_val << std::endl;
        break;
      case '-':
        pA.hue_val--;
        std::cout << "New hue val: " << pA.hue_val << std::endl;
        process_pc = true;
        break;
      case '=':
        cin >> pct;
        cin >> rct;
        segmenter.reg.setPointColorThreshold(pct);
        segmenter.reg.setRegionColorThreshold(pct);
        break;
      case ']':
        pA.saturation_mapped_value++;
        process_pc = true;
        break;
      case '[':
        pA.saturation_mapped_value--;
        process_pc = true;
        break;
      case 'h':
        system("cat commands.txt");
        break;
      case 'u':
        is_set_z_thresh = !is_set_z_thresh;
        break;
      case '0':
        separate_white = true;
        break;
      case '1':
        std::cout << "Enter new hue value: ";
        cin >> pA.hue_val;
        std::cout << "New hue val: " << pA.hue_val << std::endl;
        process_pc = true;
        break;
      default:
      break;
    }

    if(jumpOut)
      break;

    if(give_info)
    {
      std::cout << "Subject: " << subject_list[subject_num] << std::endl;
      std::cout << "Skill: "   << skill_names[skill_num] << std::endl;
      std::cout << "Demo: "    << demo_num << std::endl;
      std::cout << "KF: "      << kf_num << std::endl;
      std::cout << "Region Thresh: " << segmenter.reg.getRegionColorThreshold() << " Point Thresh: " << segmenter.reg.getPointColorThreshold() << std::endl;
      std::cout << "Observation: " << isObs << std::endl;

      if(viz_ind_clusters)
      {
          std::cout << "Selected Cluster: " << selectedCluster << std::endl;
          std::cout << "Pose: " << feats[selectedCluster].centroid[0] << " " << feats[selectedCluster].centroid[1] << " " << feats[selectedCluster].centroid[2] << " " << feats[selectedCluster].bb_orientation;
          std::cout << std::endl;
          std::cout << "Color: " << feats[selectedCluster].color[0] << " " << feats[selectedCluster].color[1] << " " << feats[selectedCluster].color[2] << " " << feats[selectedCluster].hue;
          std::cout << std::endl;
          std::cout << "Size: " << feats[selectedCluster].volume2;
          std::cout << std::endl;
      }
      if(merge_list_list.size() > 0)
      {
        for(int j = 0; j < merge_list_list.size(); j++)
        {
          std::cout << "Merge List: " << j << endl;
          for(int i = 0; i < merge_list_list[j].size(); i++)
            std::cout << merge_list_list[j][i] << " ";
          std::cout << std::endl;
        }
      }
      if(selected_clusters.size() > 0)
      {
        std::cout << "Selected Clusters List: " << endl;
        for(int i = 0; i < selected_clusters.size(); i++)
          std::cout << selected_clusters[i] << " ";
        std::cout << std::endl;
      }
      give_info = false;
    }

    if(load_pc)
    {
      cluster_num = 0;//a bad idea to do it here!

      base_skill_path.clear();
      base_skill_path.str(std::string());
      if(has_subjects)//(subject_base[0] != "")
      {
        base_skill_path << folder_path << "/" << subject_base << subject_list[subject_num] << "/" << skill_names[skill_num];
      }
      else
      {
        base_skill_path << folder_path << "/" << skill_names[skill_num] << "/" << folder_base;
      }
      std::cout << base_skill_path.str() << std::endl;
      sprintf(bsp,"%s",base_skill_path.str().c_str());

      sprintf(name,"%s/%s",bsp,processed_folder);
      makeFolder(name);

      //sprintf(name,"%s%d.%s",folder_path,subject_base,subject_num,skill_names[skill_num],demo_base,demo_num,separator,kf_base,kf_num,ext);
      std::stringstream file_path;

      file_path << bsp << "/" <<  demo_base << demo_num << separator << kf_base << kf_num << "." << ext;

      sprintf(name,"%s",file_path.str().c_str());
      if(!loadPCD(name,cloud_ptr))
      {
        load_pc = false;
        continue;
      }

      load_pc = false;
      process_pc = true;// && !isCls;
    }

    if(process_pc)
    {
      feats.clear();
      fittedBoxes.clear();
      cluster_colors.clear();
      clusters.clear();
      cluster_normals.clear();
      color_clusters.clear();
      color_cluster_normals.clear();

      selectedCluster = segmenter.run3(cloud_ptr,feats,pA.hue_val,pA.hue_thresh, pA.z_thresh, pA.euc_thresh, pA.pre_proc, pA.seg_color_ind, pA.merge_clusters, pA.displayAllBb);
      
      process_pc= false;
    }
  }

  return 0;
}



