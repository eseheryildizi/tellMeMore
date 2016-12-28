/*
 * offlineProc.cpp
 *
 *  Created on: Aug 16, 2013
 *    Author: baris
 */
#include<pc_segmentation.hpp>
#include<stdio.h>
#include<utils_pcl_ros.hpp>

int starting_subject = 0;
int starting_demo = 0;
int starting_kf = 0;
int starting_skill = 0;
bool is_set_z_thresh = false;

void readInputArgs(int argc, char *argv[])
{
  for(int i =1; i < argc; i++)
  {
    if(!strcmp(argv[i],"-n"))
    {
      starting_subject = atoi(argv[++i]);
    }
    else if(!strcmp(argv[i],"-n"))
    {
      starting_skill = atoi(argv[++i]);
    }
    else if(!strcmp(argv[i],"-d"))
    {
      starting_demo = atoi(argv[++i]);
    }
    else if(!strcmp(argv[i],"-k"))
    {
      starting_kf = atoi(argv[++i]);
    }
  }
}

int main(int argc, char *argv[])
{
  readInputArgs(argc, argv);

  bool only_color = false;
  bool no_color = !only_color && false;

  OpenNIOrganizedMultiPlaneSegmentation segmenter;
  char folder_path[] = "/home/baris/data/icra14";
  char subject_base[] = "subject";
  char *skill_names[] = {"close the box","pour"};//{"close the box", "insert", "place"};
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
  bool viz_ind_clusters = false;
  bool save_features = false;
  bool isObs = false;
  bool isCls = false;
  bool is_select_clusters = false;
  bool save_selected_cluster = false;
  bool save_euclidean_indices = false;
  bool massive_save = false;
  int selectedCluster = 0;
  int subject_num = starting_subject-1;//0;
  int skill_num = starting_skill;
  int demo_num = starting_demo;
  int kf_num = starting_kf;

  int cluster_num = 0;

  std::vector<int> selected_clusters;

  int subject_list[] = {1,2,3,4,5,6,7,8};//,9,10};

  segmenter.initSegmentation();
  segmenter.setViewer(viewer);

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

  //int merge_list[2];
  std::vector<int> merge_list;
  bool is_merge = false;
  pcl::PointCloud<PointT> merged_cluster;
  pcl::PointCloud<pcl::Normal> merged_cluster_normal;

  pcl::PointCloud<PointT> saved_cluster_xyzrgba;
  pcl::PointCloud<pcl::Normal> saved_cluster_normal;
  pcl::PointCloud<pcl::PointXYZRGBNormal> saved_cluster_all;

  std::vector<pcl::PointIndices> label_indices;

  /*demo_num = 0;
  kf_num = 5;
  segmenter.reg.setPointColorThreshold(8);
  segmenter.reg.setRegionColorThreshold(10);*/

  std::stringstream base_skill_path;
  char bsp[1024];

  isCls = true;

  int numSubjects = 8;
  int numDemos[] = {6,3};
  for(subject_num = 0; subject_num<numSubjects;subject_num++)
  {
    for(int cond = 0; cond < 2; cond++)
    {
      for(demo_num = 0; demo_num<numDemos[cond];demo_num++)
      {
        kf_num = 0;

        bool hasReaminingKFs = true;

        while(hasReaminingKFs)
        {
          if (cond == 1)
            isObs = true;
          else
            isObs = false;

          cluster_num = 0;//a bad idea to do it here!

          base_skill_path.clear();
          base_skill_path.str(std::string());
          base_skill_path << folder_path << "/" << subject_base << subject_list[subject_num] << "/" << skill_names[skill_num];
          std::cout << base_skill_path.str() << std::endl;
          sprintf(bsp,"%s",base_skill_path.str().c_str());

          /*sprintf(name,"%s/%s",bsp,processed_folder);
          makeFolder(name);*/

          //sprintf(name,"%s%d.%s",folder_path,subject_base,subject_num,skill_names[skill_num],demo_base,demo_num,separator,kf_base,kf_num,ext);
          std::stringstream file_path;

          file_path << bsp << "/";
          if(isCls)
          {
            file_path << processed_folder << "/";
          }
          if(isObs)
          {
              file_path << "observation" << separator;
          }
          file_path <<  demo_base << demo_num << separator << kf_base << kf_num;

          if(isCls)
          {
              std::stringstream file_path_normals;

              file_path_normals << file_path.str() << separator << "cluster_normals." << ext;
              file_path << separator << "clusters." << ext;

              sprintf(name,"%s",file_path.str().c_str());
              if(!loadPCD(name,cloud_ptr))
              {
                load_pc = false;
                break;
              }

              sprintf(name,"%s",file_path_normals.str().c_str());
              if(!loadPCD(name,ncloud_ptr))
              {
                load_pc = false;
                break;
              }

              if(isObs)
                sprintf(name,"%s/%s/observation_demo%d_kf%d_indices.txt",bsp,processed_folder,demo_num,kf_num);
              else
                sprintf(name,"%s/%s/demo%d_kf%d_indices.txt",bsp,processed_folder,demo_num,kf_num);

              label_indices.clear();
              std::vector< std::vector<int> > tmp_inds;
              file2doubleVector(tmp_inds,name);
              for(int i=0; i<tmp_inds.size();i++)
              {
                  pcl::PointIndices indices;
                  indices.indices = tmp_inds[i];
                  label_indices.push_back(indices);
              }

              /*pcl::PointIndices indices;
              //label_indices
              file2vector(indices.indices,name);
              label_indices.push_back(indices);*/
          }
          else
          {
            file_path << "." << ext;

            sprintf(name,"%s",file_path.str().c_str());
            if(!loadPCD(name,cloud_ptr))
            {
              load_pc = false;
              break;
            }
          }
          process_pc = true;// && !isCls;

          pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>(*cloud_ptr));
          std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;

          pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
          pcl::PointCloud<pcl::Label>::Ptr label_cloud(new pcl::PointCloud<pcl::Label>);


          feats.clear();
          fittedBoxes.clear();
          cluster_colors.clear();
          clusters.clear();
          cluster_normals.clear();
          color_clusters.clear();
          color_cluster_normals.clear();

          if(only_color)
          {
            if(!isCls)
            {
              std::vector<pcl::ModelCoefficients> model_coefficients;
              std::vector<pcl::PointIndices> inlier_indices;
              pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
              std::vector<pcl::PointIndices> label_indices_tmp;
              std::vector<pcl::PointIndices> boundary_indices;
              segmenter.planeExtract(filtered_cloud, regions, normal_cloud,model_coefficients,inlier_indices,labels,label_indices_tmp,boundary_indices);
              int closest_index;
              float closestDepth = segmenter.getClosestPlaneModel(regions, closest_index);
              bool isFilter[] = {false,false,false,false,false,true};
              float threshs[]  = {0,0,0, 0, 0, closestDepth*0.98};
              segmenter.threshXYZ(filtered_cloud, isFilter, threshs);
              segmenter.threshXYZ2(filtered_cloud, normal_cloud, isFilter, threshs); //thresh normals with the cloud
              //remove nans from the clouds!!
              std::vector< int > index;
              pcl::removeNaNFromPointCloud(*filtered_cloud,*filtered_cloud,index);
              pcl::removeNaNNormalsFromPointCloud(*normal_cloud,*normal_cloud,index);
              clusters.push_back(*filtered_cloud);
              cluster_normals.push_back(*normal_cloud);
            }
            else
            {
                clusters.push_back(*cloud_ptr);
                cluster_normals.push_back(*ncloud_ptr);
            }
          }
          else
          {
            if(!isCls)
              segmenter.segmentPointCloud(filtered_cloud, clusters, cluster_normals);
            else
              segmenter.segmentPointCloudGivenSavedCluster(cloud_ptr, ncloud_ptr, clusters, cluster_normals, label_indices);
          }

          if(no_color)
          {
            used_clusters = &clusters;
            used_cluster_normals = &cluster_normals;
          }
          else
          {
            segmenter.colorSegmentation(clusters, cluster_normals, color_clusters, color_cluster_normals);
            used_clusters = &color_clusters;
            used_cluster_normals = &color_cluster_normals;
          }

          is_merge = (used_clusters->size() > 1);

          if(is_merge)
          {
            merged_cluster.clear();
            merged_cluster_normal.clear();

            for(int i = 0; i < used_clusters->size(); i++)
            {
              merged_cluster        += (*used_clusters)[i];
              merged_cluster_normal += (*used_cluster_normals)[i];
            }

            used_clusters->push_back(merged_cluster);
            used_cluster_normals->push_back(merged_cluster_normal);

            is_merge = false;
          }
          selectedCluster = used_clusters->size()-1;

          segmenter.featureExtraction(*used_clusters, *used_cluster_normals, feats, fittedBoxes, cluster_colors);
          cout << used_clusters->size() << endl;
          cout << selectedCluster<< endl;

          save_features = true;

          if(save_features) //should find a better way to do this portion
          {
            std::cout << "Saving features selected clusters to a csv file ";
            if(isObs)
              sprintf(name,"%s/%s/observation_demo%d_kf%d.csv",bsp,processed_folder,demo_num,kf_num);
            else
              sprintf(name,"%s/%s/demo%d_kf%d.csv",bsp,processed_folder,demo_num,kf_num);
            std::cout << name << " ";
            if(massive_save)
            {
              if(isPath(name) == PT_FILE) //since we are appending
                remove(name);
              std::cout << " for " << selected_clusters.size() << " number of clusters";
              for(int i = 0; i < selected_clusters.size(); i++)
                feats[selected_clusters[i]].write2file(name, true); //append write
            }
            else
              feats[selectedCluster].write2file(name); //overwrite
            std::cout << std::endl;
            save_features = false;
          }
          kf_num++;
        }
      }
    }
  }
  return 0;
}



