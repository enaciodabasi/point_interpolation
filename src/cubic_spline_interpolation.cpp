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
    (coeffs.c * (interval_markers.first.m_x - x_to_interpolate)) +
    (coeffs.d * (interval_markers.second.m_x - x_to_interpolate)); 

  return interpolated_y;
}

std::optional<std::vector<CubicFunction>> calculateSplineFunctions(const std::vector<Point> &points, CubicSplineType type = CubicSplineType::Natural)
{
  std::size_t numDataPoints = points.size();
  std::size_t numIntervals = numDataPoints - 1;

  if (numDataPoints < 4)
  {
    return std::nullopt;
  }
  /*
    Ax = b;
  */
  eigen::Matrix<double, eigen::Dynamic, eigen::Dynamic> A;
  eigen::Vector<double, eigen::Dynamic> resultVec; // b
                                                   /*   eigen::Vector<double, eigen::Dynamic> coeffVec; // x
                                                    */
  A.setZero(numIntervals, numIntervals);
  resultVec.setZero(numIntervals);
  /*   coeffVec.setZero(numIntervals);
   */
  // 6*(Bi - Bi-1) = Hi-1*Zi-1 + 2.0*(Hi-1 + Hi)*Zi + HiZi+1;
  /*
    We have 4*numPoints - 2 equations with the basic cubic spline constraints
    So we add 2 more conditions in order to have 4*numPoints variables for calculating the coefficients:
  */

  // Calculate difference between consecutive data points:
  std::vector<double> h;
  std::vector<double> b;
  h.reserve(numIntervals);
  b.reserve(numIntervals); 
  for (std::size_t i = 0; i < numIntervals; i++)
  {
    h.at(i) = points.at(i + 1).m_x + points.at(i).m_x;
    b.at(i) = (1.0 / h.at(i)) * ((points.at(i + 1).m_y) - points.at(i).m_y);
  }

  for (std::size_t i = 1; i < numIntervals; i++)
  {
    resultVec(i) = 6.0 * (b.at(i) - b.at(i - 1));
  }

  /* *(resultVec.begin() + 0) = 0.0;
   *(resultVec.end() - 1) = 0.0; */
  resultVec(0) = 0.0;
  resultVec(numIntervals - 1) = 0.0;

  A(0, 0) = 1.0;
  A(numIntervals, 1) = 1.0;
  for (std::size_t i = 1; i < numIntervals; i++)
  {
    A(i, i - 1) = h.at(i - 1);
    A(i, i) = 2.0 * (h.at(i - 1) + h.at(i));
    A(i, i + 1) = h.at(i);
  }

  eigen::Vector<double, eigen::Dynamic> coeffVec = A.lu().solve(resultVec);

  switch (type)
  {
  case CubicSplineType::Natural:
  {
    coeffVec(0) = 0;
    coeffVec(numIntervals) = 0;
    break;
  }

  default:
    break;
  }

  /*
    Calculate polynomial coefficients for intervals 0 to numDataPoints - 1:
  */
  std::vector<CubicFunction> intervalFunctionVec;
  using namespace std::placeholders;
  for(std::size_t i = 0; i < numIntervals; i++)
  {
    CubicSplineCoefficients coeffs;
    coeffs.a = (coeffVec(i+1) / 6.0 * h.at(i));
    coeffs.b = (coeffVec(i) / 6.0 * h.at(i));
    coeffs.c = ((points.at(i+1).m_y + 1.0) / h.at(i)) - ((coeffVec(i+1) * h.at(i)) / 6.0);
    coeffs.d = (points.at(i).m_y / h.at(i)) - ((coeffVec(i)*h.at(i)) / 6.0);
    CubicFunction func = std::bind(calculateCubicFunction, coeffs, _1, _2);
    intervalFunctionVec.emplace_back(std::move(func));
  }

  return intervalFunctionVec;
}

