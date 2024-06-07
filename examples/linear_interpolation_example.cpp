/**
 * @file linear_interpolation_example.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "point_interpolation/linear_interpolation.hpp"

#include <iostream>
#include <matplot/matplot.h>

int main(int argc, char** argv)
{
  
  std::vector<double> xCoords = {
    -2.0,
    -1.0,
    1.0,
    2.0,
    3.0
  };

  std::vector<double> yCoords{
    2.0,
    1.0,
    1.5,
    2.5,
    2.25
  };

  std::vector<Point> points;
  //xCoords.resize(xCoords.size());

  int i = 0;
  for(auto xCoord : xCoords)
  {
    Point p;
    p.m_x = xCoord;
    p.m_y = yCoords.at(i);
    points.push_back(p);
    i++;
  }

  bool interpolationResult = linear_interpolation::generateInterpolatedPoints(points, 0.1);

  if(interpolationResult)
  {
    std::vector<double> X;
    std::vector<double> Y;
    for(const auto& p : points)
    {
      X.push_back(p.m_x);
      Y.push_back(p.m_y);
      std::cout << "x: " << p.m_x << " | y: " << p.m_y << std::endl;
    }
    matplot::plot(X, Y);
    matplot::show();
  }
  else
    std::cout << "Error during interpolation" << std::endl;
  return 0;


}