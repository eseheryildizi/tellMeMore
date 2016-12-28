/*
 * region_growing_custom_color.cpp
 *
 *  Created on: Nov 8, 2013
 *      Author: siml
 */

#include<region_growing_custom_color.hpp>
#include<utils.hpp>



RegionGrowingCC::RegionGrowingCC () : pcl::RegionGrowingRGB<PointT> () {}

/** \brief Destructor that frees memory. */

RegionGrowingCC::~RegionGrowingCC () {}


float
RegionGrowingCC::calculateColorimetricalDifference (std::vector<unsigned int>& first_color, std::vector<unsigned int>& second_color) const
{
  //float difference = hueDifference(first_color,second_color);

  //return (difference);
  //float h1 = rgb2hue(first_color);
  //float h2 = rgb2hue(second_color);
  //return (h1-h2)*(h1-h2);

  return ((first_color[0] - second_color[0]) * (first_color[0] - second_color[0]));
}



float
RegionGrowingCC::hueDifference (std::vector<unsigned int>& first_color, std::vector<unsigned int>& second_color)
{
  float h1 = rgb2hue(first_color);
  float h2 = rgb2hue(second_color);
  return (h1-h2)*(h1-h2);
}


float
RegionGrowingCC::rgb2hsv(std::vector<unsigned int>& rgb_color, std::vector<float>& hsv_color)
{
  const unsigned char max = std::max (rgb_color[0], std::max (rgb_color[1], rgb_color[2]));
  const unsigned char min = std::min (rgb_color[0], std::min (rgb_color[1], rgb_color[2]));

  //v
  hsv_color[2] = static_cast <float> (max) / 255.f;

  if (max == 0) // division by zero
  {
    hsv_color[1] = 0.f;
    hsv_color[0] = 0.f; //-1??
    return hsv_color[0];
  }

  const float diff = static_cast <float> (max - min);
  hsv_color[1] = diff / static_cast <float> (max);

  if (min == max) // diff == 0 -> division by zero
  {
   hsv_color[0] = 0;
   return hsv_color[0];
  }

  if      (max == rgb_color[0]) hsv_color[0] = 60.f * (      static_cast <float> (rgb_color[1] - rgb_color[2]) / diff);
  else if (max == rgb_color[1]) hsv_color[0] = 60.f * (2.f + static_cast <float> (rgb_color[2] - rgb_color[0]) / diff);
  else                  hsv_color[0] = 60.f * (4.f + static_cast <float> (rgb_color[0] - rgb_color[1]) / diff); // max == b

  if (hsv_color[0] < 0.f) hsv_color[0] += 360.f;

  return hsv_color[0];
}

