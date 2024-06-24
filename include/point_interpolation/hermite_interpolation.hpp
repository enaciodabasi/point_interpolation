/**
 * @file hermite_interpolation.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef HERMITE_INTERPOLATION_HPP_
#define HERMITE_INTERPOLATION_HPP_

#include <optional>
#include <vector>
#include <functional>
#include <iostream>
#include <cmath>

#include "geometry_utilities/point.hpp"
#include "geometry_utilities/utils.hpp"

using HermiteInterpolationFunction = std::function<double(std::pair<Point, Point>, double)>;

enum class HermiteSplineType
{
  
};

struct NonCopyableAndNonMoveable
{

  NonCopyableAndNonMoveable() = default;

  NonCopyableAndNonMoveable(const NonCopyableAndNonMoveable&) = delete;
  NonCopyableAndNonMoveable(NonCopyableAndNonMoveable&&) = delete;

  NonCopyableAndNonMoveable& operator=(const NonCopyableAndNonMoveable&) = delete;
  NonCopyableAndNonMoveable& operator=(NonCopyableAndNonMoveable&&) = delete;
};

using HermiteBasisFunction = std::function<double(double)>;

struct ExpandedHermiteBasisFunctions : private NonCopyableAndNonMoveable
{
  
  static double h00(double t);
  static double h10(double t);
  static double h01(double t);
  static double h11(double t);
  
  ExpandedHermiteBasisFunctions() = default;

};

/**
 * @brief 
 * 
 * @param points 
 * @param step_size 
 * @return true 
 * @return false 
 */
bool hermiteInterpolation(
  std::vector<Point>& points,
  double step_size
);

#endif // HERMITE_INTERPOLATION_HPP_