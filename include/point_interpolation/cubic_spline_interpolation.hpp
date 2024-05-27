/**
 * @file cubic_spline_interpolation.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef CUBIC_SPLINE_INTERPOLATION_HPP_
#define CUBIC_SPLINE_INTERPOLATION_HPP_

#include <point_interpolation/utils/geometry.hpp>

#include <eigen3/Eigen/Eigen>

#include <vector>
#include <optional>
#include <functional>

namespace eigen = Eigen;

enum class CubicSplineType
{
  Natural, // Second Derivative
  Clamped, // First Derivative
  Periodic
};

struct CubicSplineCoefficients
{

  double a;
  double b;
  double c;
  double d;

  CubicSplineCoefficients();

  ~CubicSplineCoefficients() = default;
};

/**
 * @brief 
 * 
 * @param coeffs Cubic function coefficients for the interval, should already be binded to the function.
 * @param points: The two points that mark the beginning and end of the interval 
 * @param point_to_interpolate New point with a predefined x coordinate to interpolate, 
 * @return double 
 */
/* void calculateCubicFunction(
  const CubicSplineCoefficients &coeffs, 
  std::pair<Point, Point> interval_markers, 
  Point& point_to_interpolate
);
 */
/**
 * @brief 
 * 
 * @param coeffs 
 * @param interval_markers 
 * @param x_to_interpolate 
 * @return double 
 */
double calculateCubicFunction(
  const CubicSplineCoefficients& coeffs,
  std::pair<Point, Point> interval_markers,
  double x_to_interpolate
);


using CubicFunction = std::function<double(
  std::pair<Point, Point>,
  double)>;

std::optional<std::vector<CubicFunction>> calculateSplineFunctions(const std::vector<Point> &points, CubicSplineType type = CubicSplineType::Natural);



#endif // CUBIC_SPLINE_INTERPOLATION_HPP_