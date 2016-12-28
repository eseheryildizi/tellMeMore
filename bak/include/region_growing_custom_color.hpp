/*
 * region_growing_custom_color.hpp
 *
 *  Created on: Nov 8, 2013
 *      Author: siml
 */

#ifndef REGION_GROWING_CUSTOM_COLOR_HPP_
#define REGION_GROWING_CUSTOM_COLOR_HPP_

#include <common.hpp>
#include <pcl/segmentation/region_growing_rgb.h>

//using namespace pcl;


class RegionGrowingCC : public pcl::RegionGrowingRGB<PointT>
{
public:
  RegionGrowingCC () ;

  /** \brief Destructor that frees memory. */
  virtual
  ~RegionGrowingCC ();

  //std::vector<float> hsv_color;

  /** \brief This method calculates the colorimetrical difference between two points.
    * \param[in] first_color the color of the first point
    * \param[in] second_color the color of the second point
    *
    * Override this with your desired color metric
    */
  float
  calculateColorimetricalDifference (std::vector<unsigned int>& first_color, std::vector<unsigned int>& second_color) const;

  float
  hueDifference (std::vector<unsigned int>& first_color, std::vector<unsigned int>& second_color);

  float
  rgb2hsv(std::vector<unsigned int>& rgb_color, std::vector<float>& hsv_color);

};



#endif /* REGION_GROWING_CUSTOM_COLOR_HPP_ */
