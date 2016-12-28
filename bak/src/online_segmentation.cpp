/*
 * online_segmentation.cpp
 *
 *  Created on: Aug 16, 2013
 *      Author: baris
 */

#include<pc_segmentation.hpp>

int
main (int argc, char **argv)
{
  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;
  //multi_plane_app.run (argc, argv);
/*  if(argc > 1)
  {
    COLOR_SEG = atoi(argv[1]);
  }*/

  multi_plane_app.run2 (argc, argv);
  return 0;
}


