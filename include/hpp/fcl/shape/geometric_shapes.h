/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */


#ifndef HPP_FCL_GEOMETRIC_SHAPES_H
#define HPP_FCL_GEOMETRIC_SHAPES_H

#include <boost/math/constants/constants.hpp>

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/data_types.h>
#include <string.h>

namespace hpp
{
namespace fcl
{

/// @brief Base class for all basic geometric shapes
class ShapeBase : public CollisionGeometry
{
public:
  ShapeBase() {}

  virtual ~ShapeBase () {};

  /// @brief Get object type: a geometric shape
  OBJECT_TYPE getObjectType() const { return OT_GEOM; }
};

/// @defgroup Geometric_Shapes
/// regroup class of differents types of geometric shapes.
/// @{

/// @brief Triangle stores the points instead of only indices of points
class TriangleP : public ShapeBase
{
public:
  TriangleP(const Vec3f& a_, const Vec3f& b_, const Vec3f& c_) : ShapeBase(), a(a_), b(b_), c(c_)
  {
  }

  /// @brief virtual function of compute AABB in local coordinate
  void computeLocalAABB();
  
  NODE_TYPE getNodeType() const { return GEOM_TRIANGLE; }

  Vec3f a, b, c;
};

/// @brief Center at zero point, axis aligned box
class Box : public ShapeBase
{
public:
  Box(FCL_REAL x, FCL_REAL y, FCL_REAL z) : ShapeBase(), halfSide(x/2, y/2, z/2)
  {
  }

  Box(const Vec3f& side_) : ShapeBase(), halfSide(side_/2) 
  {
  }

  Box() {}

  /// @brief box side half-length
  Vec3f halfSide;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a box
  NODE_TYPE getNodeType() const { return GEOM_BOX; }

  FCL_REAL computeVolume() const
  {
    return 8*halfSide.prod();
  }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL V = computeVolume();
    Vec3f s (halfSide.cwiseAbs2() * V);
    return (Vec3f (s[1] + s[2], s[0] + s[2], s[0] + s[1]) / 3).asDiagonal();
  }
};

/// @brief Center at zero point sphere
class Sphere : public ShapeBase
{
public:
  Sphere(FCL_REAL radius_) : ShapeBase(), radius(radius_)
  {
  }
  
  /// @brief Radius of the sphere 
  FCL_REAL radius;

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a sphere 
  NODE_TYPE getNodeType() const { return GEOM_SPHERE; }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL I = 0.4 * radius * radius * computeVolume();
    return I * Matrix3f::Identity();
  }

  FCL_REAL computeVolume() const
  {
    return 4 * boost::math::constants::pi<FCL_REAL>() * radius * radius / 3;
  }
};

/// @brief Center at zero point capsule 
class Capsule : public ShapeBase
{
public:
  Capsule(FCL_REAL radius_, FCL_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_)
  {
    lz = lz_;
    HalfLength = lz_/2;
  }

  /// @brief Radius of capsule 
  FCL_REAL radius;

  /// @brief Length along z axis 
  FCL_REAL lz;

  /// @brief Half Length along z axis 
  FCL_REAL HalfLength;

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a capsule 
  NODE_TYPE getNodeType() const { return GEOM_CAPSULE; }

  FCL_REAL computeVolume() const
  {
    return boost::math::constants::pi<FCL_REAL>() * radius * radius *(lz + radius * 4/3.0);
  }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL v_cyl = radius * radius * lz * boost::math::constants::pi<FCL_REAL>();
    FCL_REAL v_sph = radius * radius * radius * boost::math::constants::pi<FCL_REAL>() * 4 / 3.0;
    
    FCL_REAL ix = v_cyl * lz * lz / 12.0 + 0.25 * v_cyl * radius + 0.4 * v_sph * radius * radius + 0.25 * v_sph * lz * lz;
    FCL_REAL iz = (0.5 * v_cyl + 0.4 * v_sph) * radius * radius;

    return (Matrix3f() << ix, 0, 0,
                          0, ix, 0,
                          0, 0, iz).finished();
  }
  
};

/// @brief Center at zero cone 
class Cone : public ShapeBase
{
public:
  Cone(FCL_REAL radius_, FCL_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_)
  {
    lz = lz_;
    HalfLength = lz_/2;
  }

  /// @brief Radius of the cone 
  FCL_REAL radius;

  /// @brief Length along z axis 
  FCL_REAL lz;

  /// @brief Half Length along z axis 
  FCL_REAL HalfLength;

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a cone 
  NODE_TYPE getNodeType() const { return GEOM_CONE; }

  FCL_REAL computeVolume() const
  {
    return boost::math::constants::pi<FCL_REAL>() * radius * radius * lz / 3;
  }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL V = computeVolume();
    FCL_REAL ix = V * (0.1 * lz * lz + 3 * radius * radius / 20);
    FCL_REAL iz = 0.3 * V * radius * radius;

    return (Matrix3f() << ix, 0, 0,
                          0, ix, 0,
                          0, 0, iz).finished();
  }

  Vec3f computeCOM() const
  {
    return Vec3f(0, 0, -0.25 * lz);
  }
};

/// @brief Center at zero cylinder 
class Cylinder : public ShapeBase
{
public:
  Cylinder(FCL_REAL radius_, FCL_REAL lz_) : ShapeBase(), radius(radius_), lz(lz_)
  {
    lz = lz_;
    HalfLength = lz_/2;
  }
  
  /// @brief Radius of the cylinder 
  FCL_REAL radius;

  /// @brief Length along z axis 
  FCL_REAL lz;

  /// @brief Half Length along z axis 
  FCL_REAL HalfLength;

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a cylinder 
  NODE_TYPE getNodeType() const { return GEOM_CYLINDER; }

  FCL_REAL computeVolume() const
  {
    return boost::math::constants::pi<FCL_REAL>() * radius * radius * lz;
  }

  Matrix3f computeMomentofInertia() const
  {
    FCL_REAL V = computeVolume();
    FCL_REAL ix = V * (3 * radius * radius + lz * lz) / 12;
    FCL_REAL iz = V * radius * radius / 2;
    return (Matrix3f() << ix, 0, 0,
                          0, ix, 0,
                          0, 0, iz).finished();
  }
};

/// @brief Base for convex polytope.
/// @note Inherited classes are responsible for filling ConvexBase::neighbors;
class ConvexBase : public ShapeBase
{
public:

  virtual ~ConvexBase();

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a conex polytope 
  NODE_TYPE getNodeType() const { return GEOM_CONVEX; }

  /// @brief An array of the points of the polygon.
  Vec3f* points;
  int num_points;

  struct Neighbors
  {
    unsigned char count_;
    unsigned int* n_;

    unsigned char const& count () const { return count_; }
    unsigned int      & operator[] (int i)       { assert(i<count_); return n_[i]; }
    unsigned int const& operator[] (int i) const { assert(i<count_); return n_[i]; }
  };
  /// Neighbors of each vertex.
  /// It is an array of size num_points. For each vertex, it contains the number
  /// of neighbors and a list of indices to them.
  Neighbors* neighbors;

  /// @brief center of the convex polytope, this is used for collision: center is guaranteed in the internal of the polytope (as it is convex) 
  Vec3f center;

protected:
  /// @brief Constructing a convex, providing normal and offset of each polytype surface, and the points and shape topology information 
  /// \param points_ list of 3D points
  /// \param num_points_ number of 3D points
  ConvexBase(bool ownStorage, Vec3f* points_, int num_points_);

  /// @brief Copy constructor 
  /// Only the list of neighbors is copied.
  ConvexBase(const ConvexBase& other);

  unsigned int* nneighbors_;

  bool own_storage_;

private:
  void computeCenter();
};

template <typename PolygonT> class Convex;

/// @brief Half Space: this is equivalent to the Plane in ODE. The separation plane is defined as n * x = d;
/// Points in the negative side of the separation plane (i.e. {x | n * x < d}) are inside the half space and points
/// in the positive side of the separation plane (i.e. {x | n * x > d}) are outside the half space
class Halfspace : public ShapeBase
{
public:
  /// @brief Construct a half space with normal direction and offset
  Halfspace(const Vec3f& n_, FCL_REAL d_) : ShapeBase(), n(n_), d(d_)
  {
    unitNormalTest();
  }

  /// @brief Construct a plane with normal direction and offset
  Halfspace(FCL_REAL a, FCL_REAL b, FCL_REAL c, FCL_REAL d_) : ShapeBase(), n(a, b, c), d(d_)
  {
    unitNormalTest();
  }

  Halfspace() : ShapeBase(), n(1, 0, 0), d(0)
  {
  }

  FCL_REAL signedDistance(const Vec3f& p) const
  {
    return n.dot(p) - d;
  }

  FCL_REAL distance(const Vec3f& p) const
  {
    return std::abs(n.dot(p) - d);
  }

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a half space
  NODE_TYPE getNodeType() const { return GEOM_HALFSPACE; }
  
  /// @brief Plane normal
  Vec3f n;
  
  /// @brief Plane offset
  FCL_REAL d;

protected:

  /// @brief Turn non-unit normal into unit
  void unitNormalTest();
};

/// @brief Infinite plane 
class Plane : public ShapeBase
{
public:
  /// @brief Construct a plane with normal direction and offset 
  Plane(const Vec3f& n_, FCL_REAL d_) : ShapeBase(), n(n_), d(d_) 
  { 
    unitNormalTest(); 
  }
  
  /// @brief Construct a plane with normal direction and offset 
  Plane(FCL_REAL a, FCL_REAL b, FCL_REAL c, FCL_REAL d_) : ShapeBase(), n(a, b, c), d(d_)
  {
    unitNormalTest();
  }

  Plane() : ShapeBase(), n(1, 0, 0), d(0)
  {}

  FCL_REAL signedDistance(const Vec3f& p) const
  {
    return n.dot(p) - d;
  }

  FCL_REAL distance(const Vec3f& p) const
  {
    return std::abs(n.dot(p) - d);
  }

  /// @brief Compute AABB 
  void computeLocalAABB();

  /// @brief Get node type: a plane 
  NODE_TYPE getNodeType() const { return GEOM_PLANE; }

  /// @brief Plane normal 
  Vec3f n;

  /// @brief Plane offset 
  FCL_REAL d;

protected:
  
  /// @brief Turn non-unit normal into unit 
  void unitNormalTest();
};

}

} // namespace hpp

#endif
