/**
 * @file linear_interpolation.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-06-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef LINEAR_INTERPOLATION_HPP_
#define LINEAR_INTERPOLATION_HPP_

#include <point_interpolation/utils/utils_geometry.hpp>

#include <vector>
#include <optional>
namespace linear_interpolation
{

  double linearInterpolation(const Point &p1, const Point &p2, double x_to_interpolate);

  bool generateInterpolatedPoints(std::vector<Point> &points, double step_size);

} // End of namespace linear_interpolation
#endif // LINEAR_INTERPOLATION_HPP_