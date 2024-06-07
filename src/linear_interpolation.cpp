/**
 * @file linear_interpolation.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-06-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "point_interpolation/linear_interpolation.hpp"

namespace linear_interpolation
{
  double linearInterpolation(const Point &p1, const Point &p2, double x_to_interpolate)
  {
    return (p1.m_y * ((p2.m_x - x_to_interpolate) / (p2.m_x - p1.m_x))) + (p2.m_y * ((x_to_interpolate - p1.m_x) / (p2.m_x - p1.m_x)));
  }

  bool generateInterpolatedPoints(std::vector<Point> &points, double step_size)
  {
    const std::size_t numIntervals = points.size() - 1;
    const double fppEpsilon = 1e-9; // Floating point precision check constant
    auto intervalGenerationThreadFunc = [step_size, fppEpsilon](const Point &interval_start, const Point &interval_end, std::vector<Point> &newPoints)
    {
      double previousPointX = interval_start.m_x;

      for (double nextPointX = previousPointX + step_size; nextPointX < interval_end.m_x; nextPointX += step_size)
      {
        Point newPoint;
        newPoint.m_x = nextPointX;
        // Check floating point precision
        if (std::abs(newPoint.m_x - interval_end.m_x) < fppEpsilon)
        {
          continue;
        }
        newPoint.m_y = linearInterpolation(interval_start, interval_end, newPoint.m_x);
        newPoints.push_back(newPoint);
      }
    };

    std::vector<Point> newPoints;
    for (std::vector<Point>::iterator pointIter = points.begin(); pointIter != (points.end() - 1); pointIter++)
    {
      newPoints.push_back(*pointIter);
      intervalGenerationThreadFunc(*pointIter, *(std::next(pointIter)), newPoints);
    }

    points = newPoints;

    return true;
  }

} // End of namespace linear_interpolation