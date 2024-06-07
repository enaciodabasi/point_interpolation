/**
 * @file cubic_spline_interpolation.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "point_interpolation/cubic_spline_interpolation.hpp"
#include <iostream>
/* void calculateCubicFunction(
  const CubicSplineCoefficients &coeffs, 
  std::pair<Point, Point> interval_markers, 
  Point& point_to_interpolate
)
{
  point_to_interpolate.m_y = calculateCubicFunction(coeffs, interval_markers, point_to_interpolate.m_x);
} */

double calculateCubicFunction(
  const CubicSplineCoefficients& coeffs,
  std::pair<Point, Point> interval_markers,
  double x_to_interpolate
)
{
  double interpolated_y = 0.0;
  interpolated_y = 
    (coeffs.a * std::pow((x_to_interpolate - interval_markers.first.m_x), 3)) +
    (coeffs.b * std::pow((interval_markers.second.m_x - x_to_interpolate), 3)) +
    (coeffs.c * (x_to_interpolate- interval_markers.first.m_x)) +
    (coeffs.d * (interval_markers.second.m_x - x_to_interpolate)); 

  return interpolated_y;
}

std::optional<std::vector<CubicFunction>> calculateSplineFunctions(const std::vector<Point> &points, CubicSplineType type)
{
  std::size_t numDataPoints = points.size();
  std::size_t numIntervals = numDataPoints - 1;

  if (numDataPoints < 4)
  {
    std::cout << "Number of data points must be larger or equal to 4" << std::endl;
    return std::nullopt;
  }

  eigen::Matrix<double, eigen::Dynamic, eigen::Dynamic> A;
  eigen::Vector<double, eigen::Dynamic> resultVec; // b
  A.setZero(numDataPoints, numDataPoints);
  resultVec.setZero(numDataPoints);

  // 6*(Bi - Bi-1) = Hi-1*Zi-1 + 2.0*(Hi-1 + Hi)*Zi + HiZi+1;
  /*
    We have 4*numPoints - 2 equations with the basic cubic spline constraints
    So we add 2 more conditions in order to have 4*numPoints variables for calculating the coefficients:
  */

  // Calculate difference between consecutive data points:
  std::vector<double> h;
  std::vector<double> b;

  for(std::size_t i = 1; i < numDataPoints; i++)
  {
    h.push_back(points.at(i).m_x - points.at(i-1).m_x);
    //b.push_back((1.0 / h.at(i-1)) * ((points.at(i).m_y) - points.at(i-1).m_y));
  }

  for(std::size_t i = 0; i < numDataPoints-1; i++)
  {
    b.push_back((points.at(i+1).m_y - points.at(i).m_y) / h.at(i));
  }

  for (std::size_t i = 1; i < numDataPoints - 1; i++)
  {
    resultVec(i) = 6.0 * (b.at(i) - b.at(i - 1));
  }
  
  
  resultVec(0) = 0.0;
  resultVec(numDataPoints - 1) = 0.0;
  for (std::size_t i = 1; i < (numDataPoints-1); i++)
  {

    A(i, i - 1) = h.at(i  - 1);
    A(i, i) = 2.0 * (h.at(i - 1) + h.at(i));
    A(i, i + 1) = h.at(i);
    if(i+1!=numDataPoints)
    { 

    }    
  }
  A(0, 0) = 1.0;
  A(numDataPoints - 1, numDataPoints- 1) = 1.0;
  std::cout << A << std::endl;
  
  eigen::Vector<double, eigen::Dynamic> coeffVec = A.lu().solve(resultVec);
  switch (type)
  {
  case CubicSplineType::Natural:
  {
    coeffVec(0) = 0.0;
    coeffVec(numDataPoints - 1) = 0.0;
    break;
  }
  case CubicSplineType::Clamped:
  {
    
  }

  default:
    break;
  }


  /*
    Calculate polynomial coefficients for intervals 0 to numDataPoints - 1:
  */
  std::vector<CubicFunction> intervalFunctionVec;
  using namespace std::placeholders;
  for(std::size_t i = 0; i < numDataPoints; i++)
  {
    if(i+1==numDataPoints)
      break;
    CubicSplineCoefficients coeffs;
    coeffs.a = (coeffVec(i+1) / (6.0 * h.at(i)));
    coeffs.b = (coeffVec(i) / (6.0 * h.at(i)));
    coeffs.c = ((points.at(i+1).m_y) / h.at(i)) - ((coeffVec(i+1) * h.at(i)) / 6.0);
    coeffs.d = (points.at(i).m_y / h.at(i)) - ((coeffVec(i)*h.at(i)) / 6.0);
    CubicFunction func = std::bind(calculateCubicFunction, coeffs, _1, _2);
    intervalFunctionVec.emplace_back(std::move(func));
  }

  return intervalFunctionVec;
}

/* std::optional<std::vector<CubicFunction>> calculateSplineFunctions(const std::vector<Point> &points, CubicSplineType type)
{
  
} */

std::size_t getIndexOfIntervalsFirstPoint(const std::vector<Point>& points, const double x_coordinate)
{
  for(std::vector<Point>::const_iterator pointIter; pointIter != points.end(); pointIter++)
  {
    if(((*pointIter).m_x < x_coordinate) && (*std::next(pointIter)).m_x > x_coordinate)
    {
      return pointIter - points.begin();
    }
  }

  return -1;
}

bool generateInterpolatedPoints(std::vector<Point>& points, double step_size)
{
  if(points.size() < 4)
  {
    return false;
  } 

  auto intervalFunctionsOpt = calculateSplineFunctions(points);

  // Check if the resulting functions exists and 
  // the number of functions is equal to the number of intervals between points:
  if(!intervalFunctionsOpt.has_value())
  { 
    return false;
  }
  if((intervalFunctionsOpt.value().size() != (points.size() - 1)))
  {
    return false;
  }
  

  std::vector<CubicFunction>& intervalFunctions = intervalFunctionsOpt.value();
  std::vector<Point> newPoints;
  const double epsilon = 1e-9;
  for(std::vector<Point>::const_iterator pointIter = points.begin(); (pointIter != (points.end() - 1)); pointIter++)
  {

    Point previousPoint = *pointIter;
    newPoints.push_back(previousPoint);
    std::size_t currentInterval = (std::size_t)(pointIter - points.begin());
    if(std::next(pointIter) == points.end())
    {
      break;
    }
    std::pair<Point, Point> intervalPoints = std::make_pair(previousPoint, *std::next(pointIter));
    double nextPoint = (*std::next(pointIter)).m_x;
    while((previousPoint.m_x + step_size) < nextPoint)
    {
      // Create a new Point and interpolate it according to previous points x coordinate + step size
      // Then add it to the existing points vector, will be sorted later:
      
      Point newPoint;
      newPoint.m_x = previousPoint.m_x + step_size;
      bool skip = false;
      for (const auto& pt : points)
      {
          if (std::abs(newPoint.m_x - pt.m_x) < epsilon)
          {
              skip = true;
              break;
          }
      }
      if (skip)
      {
          previousPoint = *std::next(pointIter);
          continue;
      }
      auto interpolatedY = intervalFunctions.at(currentInterval)(intervalPoints, newPoint.m_x);
      newPoint.m_y = interpolatedY;
      previousPoint = newPoint;
      newPoints.push_back(newPoint);
    }
  }

  points = newPoints;
  /* std::sort(points.begin(), points.end(), [](const Point& p1, const Point& p2) -> bool {
    return (p1.m_x < p2.m_x ? true : false);
  }); */

  return true;
}

bool generateClosedCurve(std::vector<Point>& points, double step_size)
{
  /*

  */
  std::vector<double> pathLengthCumSum; // cummulative summation of path lengths 
  pathLengthCumSum.resize(points.size());
  pathLengthCumSum.at(0) = 0.0;
  
  for(std::size_t i = 1; i < points.size(); i++)
  {
    double si = pathLengthCumSum.at(i-1) + distanceBetweenTwoPoints(points.at(i-1), points.at(i));
    pathLengthCumSum.at(i) = si;
  }

  std::vector<Point> newPointsX = points;
  std::vector<Point> newPointsY = points;

  for(std::size_t i = 0; i < points.size(); i++)
  {
    newPointsX.at(i).m_y = newPointsX.at(i).m_x;
    newPointsX.at(i).m_x = pathLengthCumSum.at(i);

    newPointsY.at(i).m_x = pathLengthCumSum.at(i); 
  }

  std::cout << newPointsX.size() << std::endl;

  generateInterpolatedPoints(newPointsX, step_size);
  generateInterpolatedPoints(newPointsY, step_size);
  
  points.clear();
  points.resize(newPointsX.size());
  for(std::size_t i = 0; i < newPointsX.size(); i++)
  {
    points.at(i).m_x = newPointsX.at(i).m_y;
    points.at(i).m_y = newPointsY.at(i).m_y; 
  }
  points.at(newPointsX.size() - 1) = *points.begin();

  return true;
  
}