/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <boost/make_shared.hpp>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/geometry/polygon_operations.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/pca.h>
#include <pcl/surface/convex_hull.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <ircp.h>
#include <signal.h>

#include <rgb_euclidean_comparator.h>

#include <boost/math/distributions/normal.hpp>
#include <cmath>

#include<rotcalipers.h>

#define UNKNOWN 0
#define TWO_BY_FOUR 1
#define ONE_BY_FOUR 2
#define ONE_BY_THREE 3
#define ONE_BY_TWO 4
#define TWO_BY_TWO 5

//not yet utilized!!!
#define PLANE_SIZE 10000
#define CLUSTER_SIZE 50
#define DIST_THRESH 0.03f

#define NO_IRCP

using namespace IRCP;
using namespace std;
IRCP::Module *ircp;
unsigned char robot = SIMON_ID;
unsigned char module = PCL_MODULE_ID;
unsigned char targetModule = BEHAVIOR_MODULE_ID;
const char* bcast = "255.255.255.255";
IRCP::IndexedArray<Float> ircpData;

int myObject;

typedef pcl::PointXYZRGBA PointT;

class OpenNIOrganizedMultiPlaneSegmentation
{
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::PointCloud<PointT>::ConstPtr prev_cloud;
  boost::mutex cloud_mutex;

public:
  OpenNIOrganizedMultiPlaneSegmentation ()
  {
  }
  ~OpenNIOrganizedMultiPlaneSegmentation ()
  {
  }

  // Set up the visualization window
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  cloudViewer (pcl::PointCloud<PointT>::ConstPtr cloud)
  {
    boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (cloud, 0, 255, 0);
    viewer->addPointCloud<PointT> (cloud, single_color, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
  }

  //call back function to copy clouds
  void
  cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    if (!viewer->wasStopped ())
    {
      cloud_mutex.lock ();
      prev_cloud = cloud;
      cloud_mutex.unlock ();
    }
  }

  void
  removePreviousDataFromScreen (size_t prev_models_size)
  {
    char name[1024];
    for (size_t i = 0; i < prev_models_size; i++)
    {
      sprintf (name, "normal_%zu", i);
      viewer->removeShape (name);

      sprintf (name, "plane_%02zu", i);
      viewer->removePointCloud (name);
    }
  }

	// display each cluster in its own color
  void
  displayEuclideanClusters (const pcl::PointCloud<PointT>::CloudVectorType &clusters, 
			    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
  {
    char name[1024];
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

    for (size_t i = 0; i < clusters.size (); i++)
    {
		  sprintf (name, "cluster_%lu",i);
		  pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),red[i%6],grn[i%6],blu[i%6]);
  		if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name))
	      viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name);
	    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
	    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
    }
  }

	//////////////////////////////////////////////////////////
	// Main Loop
	//////////////////////////////////////////////////////////
  void
  run ()
  {
  // make an IRCP module
    IRCP::Module main_ircp((unsigned char)robot, module, bcast);
    ircp = &main_ircp;

    pcl::Grabber* interface = new pcl::OpenNIGrabber ();

    boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&OpenNIOrganizedMultiPlaneSegmentation::cloud_cb_, this, _1);

  //make a viewer
    pcl::PointCloud<PointT>::Ptr init_cloud_ptr (new pcl::PointCloud<PointT>);
    viewer = cloudViewer (init_cloud_ptr);
    boost::signals2::connection c = interface->registerCallback (f);

    interface->start ();

    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

	// configure normal estimation
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor (0.025f); //(0.01f);//0.03f
    ne.setNormalSmoothingSize (20.0f);//15.0f//20.0f
    
	// create a euclidean cluster comparator
    pcl::RGBEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_ = pcl::RGBEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr (new pcl::RGBEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());

	// configure the multi plane segmentor
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers (5000); //(10000);
    mps.setAngularThreshold (0.017453 * 4.0);// ); //4.5// 3 degrees
    mps.setDistanceThreshold (0.0075); //0.01 in meters
    mps.setMaximumCurvature(1000.005);//0.001

	// variable to store planar regions
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;

    pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::VectorType polygon;
    pcl::PointCloud<PointT>::VectorType approx_polygon;
    pcl::PointCloud<PointT>::VectorType approx_contour;
    size_t prev_models_size = 0;
    char name[1024];

	// while the viewer window is open
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);

      if (prev_cloud && cloud_mutex.try_lock ())
      {
        regions.clear ();
	      // Set points greater than max_thresh ot NAN
  		  float max_dist_thresh = 2.0;
	      pcl::PointCloud<PointT>::Ptr filtered_prev_cloud(new pcl::PointCloud<PointT>(*prev_cloud));
	      for (size_t i = 0; i < prev_cloud->size(); i++) 
        {
			    if (prev_cloud->points[i].z > max_dist_thresh) 
          {
  		      filtered_prev_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();//Can get away with just x
  		      //filtered_prev_cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
  		      //filtered_prev_cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
  		    }
	      }

	  // Estimate normals
	      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
	      double normal_start = pcl::getTime ();
	      ne.setInputCloud (filtered_prev_cloud);
	      ne.compute (*normal_cloud);
	      double normal_end = pcl::getTime ();
	      std::cout << "Normal Estimation took " << double (normal_end - normal_start) << std::endl;

	  // Segment planes
	      double plane_extract_start = pcl::getTime ();
	      std::vector<pcl::ModelCoefficients> model_coefficients;
	      std::vector<pcl::PointIndices> inlier_indices;  
	      pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	      std::vector<pcl::PointIndices> label_indices;
	      std::vector<pcl::PointIndices> boundary_indices;
        mps.setInputNormals (normal_cloud);
	      mps.setInputCloud (filtered_prev_cloud);
    	  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

    	  // mps.segmentAndRefine (regions);
	      double plane_extract_end = pcl::getTime ();
	      std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
	      std::cout << "Frame took " << double (plane_extract_end - normal_start) << std::endl;
	  
	  // Segment objects
    	  pcl::PointCloud<PointT>::CloudVectorType clusters;
    	  if (regions.size () > 0)
    	  {
		    // label regions with a size > 10000 as a plane?
		      std::vector<bool> plane_labels;
		      plane_labels.resize (label_indices.size (), false);
		      for (size_t i = 0; i < label_indices.size (); i++)
		      {
		        if (label_indices[i].indices.size () > 10000)
		        {
		          plane_labels[i] = true;
		        }
		      }  
	
	        euclidean_cluster_comparator_->setInputCloud (filtered_prev_cloud);
  			  euclidean_cluster_comparator_->setLabels (labels);
	        euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
	        euclidean_cluster_comparator_->setDistanceThreshold (0.03f, false);
	
	        pcl::PointCloud<pcl::Label> euclidean_labels;
	        std::vector<pcl::PointIndices> euclidean_label_indices;
	        pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
	        euclidean_segmentation.setInputCloud (filtered_prev_cloud);
	        euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
	
		      for (size_t i = 0; i < euclidean_label_indices.size (); i++)
		      {
		        if (euclidean_label_indices[i].indices.size () > 50)
		        {
		          pcl::PointCloud<PointT> cluster;
  			      pcl::copyPointCloud (*filtered_prev_cloud,euclidean_label_indices[i].indices,cluster);
	    	      clusters.push_back (cluster);
		        }
		      }
		      PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
	      }


	    // Fill and send IRCP data
	      Eigen::Vector4f min;
	      Eigen::Vector4f max;
	      int index = 1;
	      int nRegions = 0;
	      float closestDepth = 3;
	      int closestIndex = -1;
	      for (size_t i = 0; i < regions.size (); i++)
	      {
		      Eigen::Vector3f centroid = regions[i].getCentroid ();
		      Eigen::Vector4f model = regions[i].getCoefficients ();
		      if (model[3] > 3 || model[3] < 0.5) 
          {
		        // skip planes that are too far or too close
		        continue;
		      }
		      if (model[3] < closestDepth) 
          {
		        closestDepth = model[3];
		        closestIndex = i;
		      }

		      ircpData[index++] = centroid[0];
		      ircpData[index++] = centroid[1];
		      ircpData[index++] = centroid[2];
		      ircpData[index++] = model[0];
		      ircpData[index++] = model[1];
		      ircpData[index++] = model[2];
		      ircpData[index++] = model[3];	  
		
		    // Approximate polygon boundary
		      contour->points = regions[i].getContour ();
		      Eigen::Vector3f axis(model[1], -model[0], 0.0f);
		      axis.normalize ();
		      float angle = acosf (model[2]);
		      Eigen::Affine3f transform = Eigen::Translation3f (0, 0, model[3]) * Eigen::AngleAxisf (angle, axis);
		      polygon.resize(contour->points.size());
		      for (int pi = 0; pi < polygon.size (); pi++)
		      {
		        polygon[pi].getVector3fMap() = transform * contour->points[pi].getVector3fMap();
		      }
		      pcl::approximatePolygon2D<PointT> (polygon, approx_polygon, 0.02f, false, true);
		      approx_contour.resize(approx_polygon.size());
		      Eigen::Affine3f inv_transform = transform.inverse();
		      for (int pi = 0; pi < approx_polygon.size(); pi++) 
		      {
		        approx_contour[pi].getVector3fMap() = inv_transform * approx_polygon[pi].getVector3fMap();
		      }
		      PCL_INFO ("New polygon %d has %d points instead of %d \n", i, approx_polygon.size(), contour->points.size());


		      ircpData[index++] = approx_contour.size();
		      for (int pi = 0; pi < approx_contour.size(); pi++) 
		      {
		        Eigen::Vector3f point = approx_contour.at(pi).getVector3fMap();
		        ircpData[index++] = point[0];
		        ircpData[index++] = point[1];
		        ircpData[index++] = point[2];
		      }
		      nRegions++;
	      }
#ifndef NO_IRCP
	      ircpData[0] = nRegions;
	      ircp->sendto(targetModule, PlaneSegmentationData(ircpData.begin(), ircpData.end()));
	      ircp->flush_ircp();
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

	      index = 1;
	      int nClusters = 0;
	      pcl::PointCloud<PointT>::CloudVectorType myClusters;
	      //pcl::PointCloud<PointT>::CloudVectorType myProjectedClusters;
	      myClusters.clear();
	      static vector<int> clusterVolumes;
	      static vector<Eigen::Vector4f> clusterCentroids;
	      vector<string> blockNames;
	      blockNames.push_back("UNKNOWN");
	      blockNames.push_back("2x4");
	      blockNames.push_back("1x4");
	      blockNames.push_back("1x3");
	      blockNames.push_back("1x2");
	      vector<int> indices;
	      pcl::PCA<PointT> pca;
	      vector<Eigen::Matrix3f> eigenMatrix;
	      vector<Eigen::Vector3f> eigenValues;
        vector<Box3D> fittedBoxes;

	      for (int i = 0; i < clusters.size(); i++) 
	      {
		      Eigen::Vector4f centroid;
		      pcl::PointCloud<PointT> cluster = clusters[i];
		      pcl::compute3DCentroid(cluster, centroid);
		      pcl::getMinMax3D(cluster, min, max);

		    // skip clusters that are farther than the table
		      if (centroid[2] > closestDepth) 
          {
		        continue;
      		}
		    // skip clusters that are too close to simon (or part of simon)
		      if (centroid[1] > 0.2)
          {
      			continue;
      		}
	
		      indices.push_back(i);
		    // add cluster to myClusters
		      myClusters.push_back(cluster);

        //BARIS: PROJECT HERE
		      pcl::PointCloud<PointT>::Ptr projectedCluster = cluster.makeShared();
		      for(size_t j = 0; j < projectedCluster->points.size(); j++)
          {
      			projectedCluster->points[j].z = 1;
      		}

        //BARIS: DO BOX FITTING HERE
        //Start with the convex hull of the projected cluster
          pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT>);
          pcl::ConvexHull<PointT> chull;
          chull.setInputCloud (projectedCluster);
          chull.reconstruct (*cloud_hull);
        //Then call your calipers algo

          Box3D boxIt;
          boxIt = minAreaRect(projectedCluster);
          fittedBoxes.push_back(boxIt);

          //myClusters.push_back(*cloud_hull);
          
      		pca.setInputCloud(projectedCluster);
      		Eigen::Matrix3f& m = pca.getEigenVectors();
      		Eigen::Vector3f& v = pca.getEigenValues();
		
		    // Check for errors
		      if((abs(m(0,2))+abs(m(1,2)) + abs(m(2,0)) + abs(m(2,1))) > 1e-14)
          {
      			//m(0,1) = - m(0,1);
      			//m(1,1) = - m(1,1);
      			printf("\t\t\t-----> ERROR CORRECTED\n");
      		}

      		eigenMatrix.push_back(m);
      		eigenValues.push_back(v);
	

		    // Do classification
      		int blockID = estimateModel(cluster);
      		/*cout << "Cluster[" << i << "], Type: " << blockNames[blockID] << endl;
      		printf("\tCentroid: (%.2f, %.2f, %.2f)\n\tSize = %lu\n", centroid[0], centroid[1], centroid[2], cluster.size());
      		printf("\tMin: (%.2f, %.2f, %.2f)\n\tMax: (%.2f, %.2f, %.2f)\n", min[0], min[1], min[2], max[0], max[1], max[2]);
      		printf("\t\t\t\t\t\tOrientation: %f degrees\n", atan(m(0,1)/m(0,0))*180/3.14159265);*/
      		clusterVolumes.push_back(cluster.size());
      		clusterCentroids.push_back(centroid);
/*          cout << "Found eigenvectors and eigenvalues" << endl;
      		cout << "(" << v[0] << ") (" << v[1] << ") (" << v[2] << ")" << endl;
      		cout << m(0,0) << " " << m(0,1) << " " << m(0,2) << endl << m(1,0) << " " << m(1,1) << " " << m(1,2) << endl << m(2,0) << " " << m(2,1) << " " << m(2,2) << endl;
      		cout << "Angle between eigenvectors: " << acos(m(0,0)*m(0,1)+m(1,0)*m(1,1)+m(2,0)*m(2,1))*180/3.14159265 << endl << endl;*/


		    // IRCP stuff
      		ircpData[index++] = centroid[0];
      		ircpData[index++] = centroid[1];
      		ircpData[index++] = centroid[2];
      		ircpData[index++] = min[0];
      		ircpData[index++] = min[1];
      		ircpData[index++] = min[2];
      		ircpData[index++] = max[0];
      		ircpData[index++] = max[1];
      		ircpData[index++] = max[2];
		
		    // average color
		      int r = 0;
		      int g = 0;
      		int b = 0;
	      	float ra = 0.0f;
	      	float ga = 0.0f;
	      	float ba = 0.0f;
	      	float n = static_cast<float> (cluster.size());
	      	for (size_t i = 0; i < cluster.size(); i++) 
          {
      		  r += cluster[i].r;
      		  g += cluster[i].g;
      		  b += cluster[i].b;
      		  ra += (float) (cluster[i].r);
      		  ga += (float) (cluster[i].g);
      		  ba += (float) (cluster[i].b);
      		}
      		r /= cluster.size();
      		g /= cluster.size();
      		b /= cluster.size();
      		ra /= n; // (float) cluster.size();
      		ga /= n; // (float) cluster.size();
      		ba /= n; // (float) cluster.size();
      		// printf("size %d \n", cluster.size());
      		// printf("avg color int: %d %d %d \n", r, g, b);
      		// printf("avg color float: %f %f %f\n", ra, ga, ba);
      		float rf = (float)r;
      		float gf = (float)g;
      		float bf = (float)b;
      		// printf("avg color casted: %f %f %f\n",rf,gf,bf);
      		int ri = (int)ra;
      		int gi = (int)ga;
      		int bi = (int)ba;
      		// printf("avg color cast: %d %d %d \n", ri, gi, bi);
      		
      		ircpData[index++] = n;
      		ircpData[index++] = ra;
	      	ircpData[index++] = ga;
	      	ircpData[index++] = ba;
	      	// printf("RGB %d, %d, %d \n", r, g, b);
	      	// printf("RGB %f, %f, %f \n", r, g, b);
	      	nClusters++;
	      }

#ifndef NO_IRCP
	      ircpData[0] = nClusters;
	      ircp->sendto(targetModule, ClusterSegmentationData(ircpData.begin(), ircpData.end()));
	      ircp->flush_ircp();
#endif

	      if (!viewer->updatePointCloud<PointT> (prev_cloud, "cloud"))
	        viewer->addPointCloud<PointT> (prev_cloud, "cloud");

	      //viewer->removePointCloud ("normals");
	      //viewer->addPointCloudNormals<PointT,pcl::Normal>(prev_cloud, normal_cloud, 10, 0.05f, "normals");
	      //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
		    // clear the visualizer
	      removePreviousDataFromScreen (prev_models_size);

	      // Draw Visualization
	      /*for (size_t i = 0; i < regions.size (); i++)
	      {
	        Eigen::Vector3f centroid = regions[i].getCentroid ();
	        Eigen::Vector4f model = regions[i].getCoefficients ();
	        pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
	        pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
	                                           centroid[1] + (0.5f * model[1]),
	                                           centroid[2] + (0.5f * model[2]));
	        sprintf (name, "normal_%zu", i);
	        viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
	        contour->points = regions[i].getContour ();
	        sprintf (name, "plane_%02zu", i);
	        pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i], grn[i], blu[i]);
	        viewer->addPointCloud (contour, color, name);
	        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
	      }*/
	      displayEuclideanClusters (myClusters, viewer);

    //////// start using normal tabs ////////////////
		    static int numBoundingBoxes = 0; 
		    char s[1024];

     		// clear bounding boxes and arrows
		    for(int bb = 0; bb < numBoundingBoxes; bb++)
        {
			    sprintf(s, "cube %d", bb);
			    viewer->removeShape(s);
			    sprintf(s, "line %d-a", bb);
			    viewer->removeShape(s);
			    sprintf(s, "line %d-b", bb);
			    viewer->removeShape(s);
			    sprintf(s, "line %d-c", bb);
			    viewer->removeShape(s);
      	}
		    viewer->removeShape("arrow");
		    // add new bounding boxes and arrows
		    numBoundingBoxes = indices.size();

		    for(int i = 0; i < indices.size(); i++)
        {
			    pcl::PointCloud<PointT> cluster = clusters[indices[i]];

          Box3D box = fittedBoxes[i];

			    // bounding box
			    pcl::ModelCoefficients cube;
			    pcl::getMinMax3D(cluster, min, max);
			    cube.values.resize(10);
			    /*cube.values[0] = min[0]+(max[0]-min[0])/2; // Translation along the X axis
			    cube.values[1] = min[1]+(max[1]-min[1])/2; // Translation along the Y axis
			    cube.values[2] = min[2]+(max[2]-min[2])/2; // Translation along the Z axis
			    cube.values[3] = 0;
			    cube.values[4] = 0;
			    cube.values[5] = 0;
			    cube.values[6] = 1;
			    cube.values[7] = fabs(max[0]-min[0]);
			    cube.values[8] = fabs(max[1]-min[1]);
			    cube.values[9] = fabs(max[2]-min[2]);*/
          cube.values[0] = box.center.x;
			    cube.values[1] = box.center.y;
			    cube.values[2] = min[2]+(max[2]-min[2])/2;//fittedBoxes[i].center.z; //this should be maxheight/2+planeHeight
          //I am guessing the next four are the quaternions
          //assuming this is qx,qy,qz,qw
			    cube.values[3] = 0;
			    cube.values[4] = 0; 
			    cube.values[5] = sin(box.angle/2);
			    cube.values[6] = cos(box.angle/2);
          //side lengths?
			    cube.values[7] = box.size.xSize;
			    cube.values[8] = box.size.ySize;
			    cube.values[9] = fabs(max[2]-min[2]); //this should be max height!
			    sprintf(s, "cube %d", i);
			    viewer->addCube(cube, s);

          //Use the projection and the box.angle to get these, right now it will be crude
			    // arrows
			    pcl::PointXYZ p1a;
			    pcl::PointXYZ p2a;
			    p1a.x = cube.values[0];
			    p1a.y = cube.values[1];
			    p1a.z = cube.values[2];
			    p2a.x = p1a.x + cos(box.angle)*0.2;//eigenMatrix[i](0,0)*0.25;
			    p2a.y = p1a.y + sin(box.angle)*0.2;//eigenMatrix[i](1,0)*0.25;
			    p2a.z = p1a.z + 0;//eigenMatrix[i](2,0)*0.25;
			    sprintf(s, "line %d-a", i);
			    viewer->addLine(p2a, p1a, 55.0, 55.0, 55.0, s);

			    pcl::PointXYZ p1b;
			    pcl::PointXYZ p2b;
			    p1b.x = cube.values[0];
			    p1b.y = cube.values[1];
			    p1b.z = cube.values[2];
			    p2b.x = p1b.x - sin(box.angle)*0.15; //+ eigenMatrix[i](0,1)*0.15;
			    p2b.y = p1b.y + cos(box.angle)*0.15; //eigenMatrix[i](1,1)*0.15;
			    p2b.z = p1b.z + 0;//eigenMatrix[i](2,1)*0.15;
			    sprintf(s, "line %d-b", i);
			    viewer->addLine(p2b, p1b, 55.0, 55.0, 55.0, s);

			    pcl::PointXYZ p1c;
			    pcl::PointXYZ p2c;
			    p1c.x = cube.values[0];
			    p1c.y = cube.values[1];
			    p1c.z = cube.values[2];
			    p2c.x = p1c.x + 0;//eigenMatrix[i](0,2)*0.1;
			    p2c.y = p1c.y + 0;//eigenMatrix[i](1,2)*0.1;
			    p2c.z = p1c.z + 0.1;//eigenMatrix[i](2,2)*0.1;
			    sprintf(s, "line %d-c", i);
			    viewer->addLine(p2c, p1c, 55.0, 55.0, 55.0, s);
	
		    }

	      /*calcSimpleStatistics(clusterVolumes, clusterCentroids);
	      char c = getchar();
	      if(c == 'c'){
		    clusterVolumes.clear();
		    clusterCentroids.clear();
		    printf("Cleared clusters!\n");
	      }
	      */

	      prev_models_size = regions.size ();
	      cloud_mutex.unlock ();
	    }
    }

    interface->stop ();
  }
	
	// Do some simple statistics calculation to build a model of the object
	void 
  calcSimpleStatistics(vector<int> volumes, vector<Eigen::Vector4f> centroids)
  {
		float mean = 0.0;
		float var = 0.0;
		for(int i = 0; i < volumes.size(); i++){
			mean += (float) volumes[i]/((float) volumes.size());
		}
		for(int i = 0; i < volumes.size(); i++){
			var += (1.0/((float) volumes.size()))*((float)volumes[i]-mean)*((float)volumes[i]-mean);
		}

		printf("Poses = %lu\nMean volume = %f\nVariance = %f\n", volumes.size(), mean, var);
	}
	
  int 
  estimateModel(pcl::PointCloud<PointT> cluster)
  {
		double baselineConfidence = 0.2;
		double confidence = 0.0;
		double highestConfidence = 0.0;
		int bestGuess = UNKNOWN;
		boost::math::normal volumeDist2x4(1904.18, 146.05);
		boost::math::normal volumeDist1x4(965.72, 170.02);
		boost::math::normal volumeDist1x3(723.629, 127.95);
		boost::math::normal volumeDist1x2(479.45, 49.37);
		
		// calculate probabilities		
		confidence = 1 - abs(boost::math::cdf(volumeDist2x4, cluster.size()) - boost::math::cdf(complement(volumeDist2x4, cluster.size())));
		if(confidence > highestConfidence){
			bestGuess = TWO_BY_FOUR;
			highestConfidence = confidence;
		}
		confidence = 1 - abs(boost::math::cdf(volumeDist1x4, cluster.size()) - boost::math::cdf(complement(volumeDist1x4, cluster.size())));
		if(confidence > highestConfidence){
			bestGuess = ONE_BY_FOUR;
			highestConfidence = confidence;
		}
		confidence = 1 - abs(boost::math::cdf(volumeDist1x3, cluster.size()) - boost::math::cdf(complement(volumeDist1x3, cluster.size())));
		if(confidence > highestConfidence){
			bestGuess = ONE_BY_THREE;
			highestConfidence = confidence;
		}
		confidence = 1 - abs(boost::math::cdf(volumeDist1x2, cluster.size()) - boost::math::cdf(complement(volumeDist1x2, cluster.size())));
		if(confidence > highestConfidence){
			bestGuess = ONE_BY_TWO;
			highestConfidence = confidence;
		}

		printf("Confidence = %f\n", highestConfidence);
		if(highestConfidence > baselineConfidence)
			return bestGuess;
		else
			return UNKNOWN;
  }

};

int
main ()
{
  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;
  multi_plane_app.run ();
#ifndef NO_IRCP
  ircp->quit();
#endif
  return 0;
}
