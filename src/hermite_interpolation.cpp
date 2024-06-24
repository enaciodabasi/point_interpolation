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

double ExpandedHermiteBasisFunctions::h00(double t)
{
  return (
      (2.0 * std::pow(t, 3)) -
      (3.0 * std::pow(t, 2)) +
      1.0
    );
}

double ExpandedHermiteBasisFunctions::h10(double t)
{
  return (
      std::pow(t, 3) - 
      (2.0 * std::pow(t, 2)) + 
      t
    );
}


double ExpandedHermiteBasisFunctions::h01(double t)
{
  return (
      (-2.0 * std::pow(t, 3)) +
      3.0 * std::pow(t, 2)
    );
}


double ExpandedHermiteBasisFunctions::h11(double t)
{
  return (
      std::pow(t, 3) -
      std::pow(t, 2)
    );
}


bool hermiteInterpolation(
  std::vector<Point>& points,
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
  
  for(std::vector<std::pair<Point, Point>>::const_iterator intervalIt = intervals.begin(); intervalIt != intervals.end(); intervalIt++)
  {
    const std::pair<Point, Point> currentInterval = *intervalIt;
    interpolatedPoints.push_back(currentInterval.first);
    double currentPoint = currentInterval.first.m_x;
    std::size_t currentIntervalIndex = (std::size_t)std::distance(intervals.cbegin(), intervalIt);

    while(currentPoint + step_size < currentInterval.second.m_x)
    {
      Point newPoint;
      newPoint.m_x = currentPoint + step_size;
      bool skip = false;
      for(const auto& pt : points)
      if(std::abs(newPoint.m_x - pt.m_x) < fppEpsilon)
      {
        skip = true;
        break;
      }
      if(skip)
      {
        
        break;
      }

      const double t = (newPoint.m_x - currentInterval.first.m_x) / (currentInterval.second.m_x - currentInterval.first.m_x); 
      double h00, h01, h10, h11 = 0.0;
      double mk, mkn = 0.0;

      h00 = ExpandedHermiteBasisFunctions::h00(t);
      h01 = ExpandedHermiteBasisFunctions::h01(t);
      h10 = ExpandedHermiteBasisFunctions::h10(t);
      h11 = ExpandedHermiteBasisFunctions::h11(t);
      
      // If finite difference is used to calculate the curve tangent
      // The intervals from the endpoints need to be calculated by one-sided difference instead of finite.
      if (currentIntervalIndex == 0)
      {
        std::cout << "First interval\n";
        mk = (currentInterval.second.m_y - currentInterval.first.m_y) / (currentInterval.second.m_x - currentInterval.first.m_x);
        mkn = (std::next(intervalIt)->first.m_y - currentInterval.second.m_y) / (std::next(intervalIt)->second.m_x - currentInterval.second.m_x); 
      }
      else if(currentIntervalIndex == points.size() - 1 - 1)
      {
        std::cout << "Last interval" << std::endl;
        mk = (currentInterval.second.m_y - currentInterval.first.m_y) / (currentInterval.second.m_x - currentInterval.first.m_x);
        mkn = 0.0;
      }
      else // Three-point difference
      {
        std::cout << "Inbetween interval\n";
        if(std::next(intervalIt) == intervals.end() || std::prev(intervalIt) == intervals.end())
        {
          break;
        }

        const Point& nextPoint = std::next(intervalIt)->second;
        const Point& previousPoint = std::prev(intervalIt)->first;
        
        mkn = 0.5 *
        (((nextPoint.m_y - currentInterval.second.m_y) / (nextPoint.m_x - currentInterval.second.m_x)) + 
        ((currentInterval.second.m_y - currentInterval.first.m_y) / (currentInterval.second.m_x - currentInterval.first.m_x)));
        
        mk = 0.5 *
        (currentInterval.second.m_y - currentInterval.first.m_y) / (currentInterval.second.m_x - currentInterval.first.m_x) +
        (currentInterval.first.m_y - previousPoint.m_y) / (currentInterval.first.m_x - previousPoint.m_x);  

        std::cout << mkn << " " << mk << std::endl;

      }
      
      double interpolatedY = 
      (h00 * currentInterval.first.m_y) + 
      (h10 *(currentInterval.second.m_x - currentInterval.first.m_x) * mk) + 
      (h01 * currentInterval.second.m_y) +
      (h11 * (currentInterval.second.m_x - currentInterval.first.m_x) * mkn);
      newPoint.m_y = interpolatedY;
      interpolatedPoints.push_back(newPoint);
      currentPoint = newPoint.m_x;
    } 
    
  }

  points = interpolatedPoints;
  return true;

}