/**
 * @file geometry.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef UTILS_GEOMETRY_HPP_
#define UTILS_GEOMETRY_HPP_

#include <math.h>
#include <utility>
struct RPY;

  struct Quaternion
  {
    public:

    Quaternion();
    
    Quaternion(double x, double y, double z, double w);

    ~Quaternion() = default;

    Quaternion(const Quaternion& other);

    Quaternion operator=(const Quaternion& other);

    Quaternion(Quaternion&& other);

    Quaternion& operator=(Quaternion&& other);

    double m_x;
    double m_y;
    double m_z;
    double m_w;

    static Quaternion fromRPY(const RPY& rpy);
    RPY toRPY() const;
  };

  struct Point
  {
    public:
    
    Point();
    
    Point(double x, double y, Quaternion orientation);

    Point(const Point& other);

    Point operator=(const Point& other);

    Point(Point&& other);

    Point& operator=(Point&& other);

    ~Point() = default;

    double m_x;
    double m_y;
    Quaternion m_Orientation;
  };

  struct RPY
  {
    public:

    RPY();

    RPY(double roll, double pitch, double yaw);

    ~RPY() = default;

    RPY(const RPY& other);

    RPY operator=(const RPY& other);

    RPY(RPY&& other);

    RPY& operator=(RPY&& other);

    double m_roll;
    double m_pitch;
    double m_yaw;

    static RPY fromQuaternion(const Quaternion& quaternion);
    Quaternion toQuaternion() const;
  };


#endif // UTILS_GEOMETRY_HPP_