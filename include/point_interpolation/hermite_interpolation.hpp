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

#include "point_interpolation/utils/utils_geometry.hpp"

using HermiteInterpolationFunction = std::function<double(std::pair<Point, Point>, double)>;

enum class HermiteSplineType
{
  
};

struct HermiteBasisFunctions
{
  using HermiteBasisFunction = std::function<double(double)>; 
  
  static HermiteBasisFunction h00;
  static HermiteBasisFunction h10;
  static HermiteBasisFunction h01;
  static HermiteBasisFunction h11;
};

struct ExpandedHermiteBasisFunctions : public HermiteBasisFunctions
{
  ExpandedHermiteBasisFunctions(){
  
  h00 = [](double t) -> double
  {
    return (
      (2.0 * std::pow(t, 3)) -
      (3.0 * std::pow(t, 2)) +
      1.0
    );
  };

  h10 = [](double t) -> double
  {
    return (
      std::pow(t, 3) - 
      (2.0 * std::pow(t, 2)) + 
      t
    );
  };

  h01 = [](double t) -> double
  {
    return (
      (-2.0 * std::pow(t, 3)) +
      3.0 * std::pow(t, 2)
    );
  };

  h11 = [](double t) -> double
  {
    return (
      std::pow(t, 3) -
      std::pow(t, 2)
    );
  };

  }
};

/**
 * @brief 
 * 
 * @param points 
 * @param type 
 * @return std::optional<std::vector<HermiteInterpolationFunction>> 
 */
std::optional<std::vector<HermiteInterpolationFunction>> calculateHermiteInterpolationFunctions(
  const std::vector<Point>& points,
  HermiteSplineType type
);

#endif // HERMITE_INTERPOLATION_HPP_