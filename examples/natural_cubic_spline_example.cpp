/**
 * @file natural_cubic_spline_example.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <point_interpolation/utils/geometry.hpp>
#include "point_interpolation/cubic_spline_interpolation.hpp"

#include <iostream>
#

int main(int argc, char** argv)
{
  
  std::vector<double> xCoords = {
    0.0,
    1.0,
    2.5,
    3.7,
    4.2,
    5.6
  };

  std::vector<double> yCoords{
    0.5,
    0.7, 
    0.9,
    1.1,
    1.3,
    1.5
  };

  std::vector<Point> points;
  //xCoords.resize(xCoords.size());

  int i = 0;
  for(auto xCoord : xCoords)
  {
    /* std::cout << xCoord << std::endl; */
    Point p;
    p.m_x = xCoord;
    p.m_y = yCoords.at(i);
    points.push_back(p);
    i++;
  }

  bool interpolationResult = generateInterpolatedPoints(points, 0.1);

  if(interpolationResult)
    for(auto p : points)
    {
      std::cout << "x: " << p.m_x << " | y: " << p.m_y << std::endl;
    }
  else
    std::cout << "Error during interpolation" << std::endl;
  return 0;


}