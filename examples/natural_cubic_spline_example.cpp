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

#include "point_interpolation/cubic_spline_interpolation.hpp"

#include <iostream>
#include <matplot/matplot.h>

int main(int argc, char** argv)
{
  std::cout << "Generating Spline:\n";
  std::vector<double> xCoords = {
    -2.0,
    0.0,
    2.0,
    0.0,
    5.0,
    -2.0
  };

  std::vector<double> yCoords{
    0.0,
    2.0,
    0.0,
    -2.0,
    3.5,
    0.0
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

  bool interpolationResult = generateClosedCurve(points, 0.1);

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
    matplot::hold(matplot::on);
    matplot::plot(xCoords, yCoords, "-o")->marker_size(10.0);
    /* matplot::scatter(xCoords, yCoords)->marker_size(10.0); */
    matplot::show();
  }
  else
    std::cout << "Error during interpolation" << std::endl;
  return 0;


}