/**
 * @file hermite_interpolation.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "point_interpolation/hermite_interpolation.hpp"

/* std::optional<std::vector<HermiteInterpolationFunction>> calculateHermiteInterpolationFunctions(
  const std::vector<Point>& points,
  HermiteSplineType type
)
{

} */

bool hermiteInterpolation(
  const std::vector<Point>& points,
  double step_size
)
{
  std::vector<std::pair<Point, Point>> intervals;

  for(std::vector<Point>::const_iterator pointIt = points.cbegin(); pointIt != points.end() - 1; pointIt++)
  {

    intervals.push_back(
      std::make_pair(
      *(pointIt),
      *std::next(pointIt)
      )
    );
  }

  if(intervals.size() != points.size() - 1)
  {
    return false;
  }

  std::vector<Point> interpolatedPoints;
  const double fppEpsilon  = 1e-9;
  static HermiteBasisFunctions* funcs;
  for(std::vector<std::pair<Point, Point>>::const_iterator intervalIt = intervals.begin(); intervalIt != intervals.end(); intervalIt++)
  {
    const std::pair<Point, Point> currentInterval = *intervalIt;
    interpolatedPoints.push_back(currentInterval.first);
    double currentPoint = currentInterval.first.m_x;
    while(currentPoint + step_size < currentInterval.second.m_x)
    {
      Point newPoint;
      newPoint.m_x = currentPoint + step_size;
      if(std::abs(newPoint.m_x - currentInterval.second.m_x) < fppEpsilon)
      {
        continue;
      }

      const double t = (newPoint.m_x - currentInterval.first.m_x) / (currentInterval.second.m_x - currentInterval.first.m_x); 
      double h00, h01, h10, h11 = 0.0;
      double mk, mkn = 0.0;

      h00 = ExpandedHermiteBasisFunctions::h00(t);
      h01 = ExpandedHermiteBasisFunctions::h01(t);
      h10 = ExpandedHermiteBasisFunctions::h10(t);
      h11 = ExpandedHermiteBasisFunctions::h11(t);
      std::size_t currentIntervalIndex = (std::size_t)std::distance(intervals.cbegin(), intervalIt);
      
      // If finite difference is used to calculate the curve tangent
      // The intervals from the endpoints need to be calculated by one-sided difference instead of finite.
      if (currentIntervalIndex == 0)
      {
        mk = (currentInterval.second.m_y - currentInterval.first.m_y) / (currentInterval.second.m_x - currentInterval.first.m_x);
        mkn = (std::next(intervalIt)->first.m_y - currentInterval.second.m_y) / (std::next(intervalIt)->second.m_x - currentInterval.second.m_x); 
      }
      else if(currentIntervalIndex == points.size() - 1)
      {
        mk = (currentInterval.second.m_y - currentInterval.first.m_y) / (currentInterval.second.m_x - currentInterval.first.m_x);
        mkn = 0.0;
      }
      else // Three-point difference
      {
        const Point& nextPoint = std::next(intervalIt)->first;
        const Point& previousPoint = std::prev(intervalIt)->first;
        
        mkn = 0.5 *
        (((nextPoint.m_y - currentInterval.second.m_y) / (nextPoint.m_x - currentInterval.second.m_x)) + 
        ((currentInterval.second.m_y - currentInterval.first.m_y) / (currentInterval.second.m_x - currentInterval.first.m_x)));
        
        mk = 0.5 *
        (currentInterval.second.m_y - currentInterval.first.m_y) / (currentInterval.second.m_x - currentInterval.first.m_x) +
        (currentInterval.first.m_y - previousPoint.m_y) / (currentInterval.first.m_x - previousPoint.m_x);  

      }
      
      
      
    } 
  }

}