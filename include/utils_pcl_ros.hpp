/*
Copyright (c) 2016, Baris Akgun
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Koc University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#ifndef UTILS_PCL_ROS_HPP_
#define UTILS_PCL_ROS_HPP_

#include<utils.hpp>
#include<vector>
#include<objectFeatures.hpp>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <PcFeatures.h>
#include <tf2_ros/transform_broadcaster.h>

/*
 * Ties are resolved by the cluster size
 */
int findFeatVecByHue(std::vector<pc_cluster_features> &feats, double main_hue);

int getClusterByColor(std::vector<pc_cluster_features> &feats, int feature_number, bool is_max = true);

void getClusterByColorThresh(std::vector<pc_cluster_features> &feats, int feature_number, std::vector<int > &indices, unsigned char low = 0, unsigned char high = 255);

// Some useful functions
void
Box3D_2_MC(Box3D &in_box, pcl::ModelCoefficients &out_cube);

void
minAreaRect(pcl::PointCloud<PointT>::Ptr &input_cloud, pcl::ModelCoefficients &out_cube);

void
Cube_2_Arrows(pcl::ModelCoefficients &cube, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int index);

Box3D
//boundingBoxWithCoeff(pcl::PointCloud<PointT> &cluster, pcl::ModelCoefficients::Ptr coefficients);
boundingBoxWithCoeff(pcl::PointCloud<PointT> &cluster, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<PointT>::Ptr &cloud_transformed);

void
fillRosMessage (pc_segmentation::PcFeatures &outRosMsg, const pc_cluster_features &inObjFeatures);

void
getObjectMarker(visualization_msgs::Marker &marker, pc_segmentation::PcFeatures &feats);

void 
objectPoseTF(geometry_msgs::Transform geom_transform);

template<class pcType>
bool
loadPCD(char *name, pcType cloud_ptr)
{
  if (isPath(name) != PT_FILE)
  {
      std::cout << "Filename " << name << " is not a valid file."
          << std::endl;
      return false;
  }
  bool result = (pcl::io::loadPCDFile(name, *cloud_ptr) == 0);
  if (result)
    std::cout << "Loaded file " << name << " with " << cloud_ptr->size()
        << " points.";
  std::cout << std::endl;
  return result;
}

#endif /* UTILS_PCL_ROS_HPP_ */
