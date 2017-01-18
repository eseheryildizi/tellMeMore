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

#ifndef MISC_STRUCTURES_HPP_
#define MISC_STRUCTURES_HPP_

#include <math.h>

//why print2cout than friending <<? Dunno, there was a silly bug.
class sizeS{
public:
  float xSize;
  float ySize;
  float zSize;

  sizeS() : xSize(0), ySize(0), zSize(0) {};

  void print2cout()
  {
    std::cout << "Sizes:" << std::endl << " x: " << xSize << " y: " << ySize << " z: " << zSize << std::endl;;
  }

  float getVolume() {
    return xSize*ySize*zSize;}

  float getArea() {
    return 2*(xSize*ySize + xSize*ySize + ySize*zSize + zSize*xSize);}

  float getAspectRatio() {
    return (ySize/xSize);}
} ;

class centerS{
public:
  float x;
  float y;
  float z;

  centerS() : x(0), y(0), z(0) {};

  void print2cout()
  {
    std::cout << "Center Coordinates:" << std::endl << " x: " << x << " y: " << y << " z: " << z << std::endl;
  }
};

class Box3D
{
public:
  sizeS size;
  centerS center;
  float angle;
  float rot_quat[4];
  float rot_axis[3];
  bool is_quat_scalar_last; //if true [qx,qy,qz,qw], else [qw,qx,qy,qz]
  float aspect_ratio;

  float volume;
  float area;

  Box3D() : size(), center(), angle(0)
  {
    is_quat_scalar_last = true;

    //default z axis
    rot_axis[0] = 0;
    rot_axis[1] = 0;
    rot_axis[2] = 1;

    volume = 0;
    area = 0;
    aspect_ratio = 1;
  };

  void fillQuatGivenAxisAngle()
  {
	std::cout << "Will need to be rewamped. Split up angle into angle wrt normal plus angle for the global rotation" << std::endl;
	float sinHalfAngle = sin(angle/2);
    if (is_quat_scalar_last)
    {
        rot_quat[0] = rot_axis[0]*sinHalfAngle;
        rot_quat[1] = rot_axis[1]*sinHalfAngle;
        rot_quat[2] = rot_axis[2]*sinHalfAngle;
        rot_quat[3] = cos(angle/2);
    }
    else
    {
        rot_quat[1] = rot_axis[0]*sinHalfAngle;
        rot_quat[2] = rot_axis[1]*sinHalfAngle;
        rot_quat[3] = rot_axis[2]*sinHalfAngle;
        rot_quat[0] = cos(angle/2);
    }

    //std::cout << rot_quat[0] << " " <<  rot_quat[1] << " " <<  rot_quat[2] << " " <<  rot_quat[3] << std::endl;
    //std::cout << rot_axis[0] << " " <<  rot_axis[1] << " " <<  rot_axis[2] <<  std::endl << std::endl;
  }

  void normalizeAxis()
  {
    float norm = 0;
    for(int i=0; i < 3; i++)
      norm += rot_axis[i]*rot_axis[i];

    norm = sqrt(norm);

    for(int i=0; i < 3; i++)
      rot_axis[i] = rot_axis[i]/norm;
  }

  void calculateProperties()
  {
    volume = size.getVolume();
    area = size.getArea();
    aspect_ratio = size.getAspectRatio();
  }

  void print2cout()
  {
    std::cout << "Box:" << std::endl;
    center.print2cout();
    size.print2cout();
    std::cout << "Angle with Z: " << angle << std::endl;

  }

//  friend std::ostream& operator<< (std::ostream& output, const Box3D& a)
//  {
//    output << "Box:" << std::endl;
//    output << a.center << std::endl;
//    output << a.size << std::endl;
//    output << "Angle with Z: " << a.angle;
//  }
};

#endif /* MISC_STRUCTURES_HPP_ */
