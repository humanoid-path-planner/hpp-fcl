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

namespace hpp {
namespace fcl {

/// @brief Base class for all basic geometric shapes
class HPP_FCL_DLLAPI ShapeBase : public CollisionGeometry {
 public:
  ShapeBase() {}

  ///  \brief Copy constructor
  ShapeBase(const ShapeBase& other) : CollisionGeometry(other) {}

  ShapeBase& operator=(const ShapeBase& other) = default;

  virtual ~ShapeBase(){};

  /// @brief Get object type: a geometric shape
  OBJECT_TYPE getObjectType() const { return OT_GEOM; }
};

/// @defgroup Geometric_Shapes Geometric shapes
/// Classes of different types of geometric shapes.
/// @{

/// @brief Triangle stores the points instead of only indices of points
class HPP_FCL_DLLAPI TriangleP : public ShapeBase {
 public:
  TriangleP(){};

  TriangleP(const Vec3f& a_, const Vec3f& b_, const Vec3f& c_)
      : ShapeBase(), a(a_), b(b_), c(c_) {}

  TriangleP(const TriangleP& other)
      : ShapeBase(other), a(other.a), b(other.b), c(other.c) {}

  /// @brief Clone *this into a new TriangleP
  virtual TriangleP* clone() const { return new TriangleP(*this); };

  /// @brief virtual function of compute AABB in local coordinate
  void computeLocalAABB();

  NODE_TYPE getNodeType() const { return GEOM_TRIANGLE; }

  //  std::pair<ShapeBase*, Transform3f> inflated(const FCL_REAL value) const {
  //    if (value == 0) return std::make_pair(new TriangleP(*this),
  //    Transform3f()); Vec3f AB(b - a), BC(c - b), CA(a - c); AB.normalize();
  //    BC.normalize();
  //    CA.normalize();
  //
  //    Vec3f new_a(a + value * Vec3f(-AB + CA).normalized());
  //    Vec3f new_b(b + value * Vec3f(-BC + AB).normalized());
  //    Vec3f new_c(c + value * Vec3f(-CA + BC).normalized());
  //
  //    return std::make_pair(new TriangleP(new_a, new_b, new_c),
  //    Transform3f());
  //  }
  //
  //  FCL_REAL minInflationValue() const
  //  {
  //    return (std::numeric_limits<FCL_REAL>::max)(); // TODO(jcarpent):
  //    implement
  //  }

  Vec3f a, b, c;

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const TriangleP* other_ptr = dynamic_cast<const TriangleP*>(&_other);
    if (other_ptr == nullptr) return false;
    const TriangleP& other = *other_ptr;

    return a == other.a && b == other.b && c == other.c;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Center at zero point, axis aligned box
class HPP_FCL_DLLAPI Box : public ShapeBase {
 public:
  Box(FCL_REAL x, FCL_REAL y, FCL_REAL z)
      : ShapeBase(), halfSide(x / 2, y / 2, z / 2) {}

  Box(const Vec3f& side_) : ShapeBase(), halfSide(side_ / 2) {}

  Box(const Box& other) : ShapeBase(other), halfSide(other.halfSide) {}

  Box& operator=(const Box& other) {
    if (this == &other) return *this;

    this->halfSide = other.halfSide;
    return *this;
  }

  /// @brief Clone *this into a new Box
  virtual Box* clone() const { return new Box(*this); };

  /// @brief Default constructor
  Box() {}

  /// @brief box side half-length
  Vec3f halfSide;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a box
  NODE_TYPE getNodeType() const { return GEOM_BOX; }

  FCL_REAL computeVolume() const { return 8 * halfSide.prod(); }

  Matrix3f computeMomentofInertia() const {
    FCL_REAL V = computeVolume();
    Vec3f s(halfSide.cwiseAbs2() * V);
    return (Vec3f(s[1] + s[2], s[0] + s[2], s[0] + s[1]) / 3).asDiagonal();
  }

  FCL_REAL minInflationValue() const { return -halfSide.minCoeff(); }

  /// \brief Inflate the box by an amount given by value
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated box and the related transform to account for the
  /// change of shape frame
  std::pair<Box, Transform3f> inflated(const FCL_REAL value) const {
    if (value <= minInflationValue())
      HPP_FCL_THROW_PRETTY("value (" << value << ") "
                                     << "is two small. It should be at least: "
                                     << minInflationValue(),
                           std::invalid_argument);
    return std::make_pair(Box(2 * (halfSide + Vec3f::Constant(value))),
                          Transform3f());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Box* other_ptr = dynamic_cast<const Box*>(&_other);
    if (other_ptr == nullptr) return false;
    const Box& other = *other_ptr;

    return halfSide == other.halfSide;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Center at zero point sphere
class HPP_FCL_DLLAPI Sphere : public ShapeBase {
 public:
  /// @brief Default constructor
  Sphere() {}

  explicit Sphere(FCL_REAL radius_) : ShapeBase(), radius(radius_) {}

  Sphere(const Sphere& other) : ShapeBase(other), radius(other.radius) {}

  /// @brief Clone *this into a new Sphere
  virtual Sphere* clone() const { return new Sphere(*this); };

  /// @brief Radius of the sphere
  FCL_REAL radius;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a sphere
  NODE_TYPE getNodeType() const { return GEOM_SPHERE; }

  Matrix3f computeMomentofInertia() const {
    FCL_REAL I = 0.4 * radius * radius * computeVolume();
    return I * Matrix3f::Identity();
  }

  FCL_REAL computeVolume() const {
    return 4 * boost::math::constants::pi<FCL_REAL>() * radius * radius *
           radius / 3;
  }

  FCL_REAL minInflationValue() const { return -radius; }

  /// \brief Inflate the sphere by an amount given by value
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated sphere and the related transform to account for
  /// the change of shape frame
  std::pair<Sphere, Transform3f> inflated(const FCL_REAL value) const {
    if (value <= minInflationValue())
      HPP_FCL_THROW_PRETTY(
          "value (" << value << ") is two small. It should be at least: "
                    << minInflationValue(),
          std::invalid_argument);
    return std::make_pair(Sphere(radius + value), Transform3f());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Sphere* other_ptr = dynamic_cast<const Sphere*>(&_other);
    if (other_ptr == nullptr) return false;
    const Sphere& other = *other_ptr;

    return radius == other.radius;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Ellipsoid centered at point zero
class HPP_FCL_DLLAPI Ellipsoid : public ShapeBase {
 public:
  /// @brief Default constructor
  Ellipsoid() {}

  Ellipsoid(FCL_REAL rx, FCL_REAL ry, FCL_REAL rz)
      : ShapeBase(), radii(rx, ry, rz) {}

  explicit Ellipsoid(const Vec3f& radii) : radii(radii) {}

  Ellipsoid(const Ellipsoid& other) : ShapeBase(other), radii(other.radii) {}

  /// @brief Clone *this into a new Ellipsoid
  virtual Ellipsoid* clone() const { return new Ellipsoid(*this); };

  /// @brief Radii of the Ellipsoid (such that on boundary: x^2/rx^2 + y^2/ry^2
  /// + z^2/rz^2 = 1)
  Vec3f radii;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: an ellipsoid
  NODE_TYPE getNodeType() const { return GEOM_ELLIPSOID; }

  Matrix3f computeMomentofInertia() const {
    FCL_REAL V = computeVolume();
    FCL_REAL a2 = V * radii[0] * radii[0];
    FCL_REAL b2 = V * radii[1] * radii[1];
    FCL_REAL c2 = V * radii[2] * radii[2];
    return (Matrix3f() << 0.2 * (b2 + c2), 0, 0, 0, 0.2 * (a2 + c2), 0, 0, 0,
            0.2 * (a2 + b2))
        .finished();
  }

  FCL_REAL computeVolume() const {
    return 4 * boost::math::constants::pi<FCL_REAL>() * radii[0] * radii[1] *
           radii[2] / 3;
  }

  FCL_REAL minInflationValue() const { return -radii.minCoeff(); }

  /// \brief Inflate the ellipsoid by an amount given by value
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated ellipsoid and the related transform to account for
  /// the change of shape frame
  std::pair<Ellipsoid, Transform3f> inflated(const FCL_REAL value) const {
    if (value <= minInflationValue())
      HPP_FCL_THROW_PRETTY(
          "value (" << value << ") is two small. It should be at least: "
                    << minInflationValue(),
          std::invalid_argument);
    return std::make_pair(Ellipsoid(radii + Vec3f::Constant(value)),
                          Transform3f());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Ellipsoid* other_ptr = dynamic_cast<const Ellipsoid*>(&_other);
    if (other_ptr == nullptr) return false;
    const Ellipsoid& other = *other_ptr;

    return radii == other.radii;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Capsule
/// It is \f$ { x~\in~\mathbb{R}^3, d(x, AB) \leq radius } \f$
/// where \f$ d(x, AB) \f$ is the distance between the point x and the capsule
/// segment AB, with \f$ A = (0,0,-halfLength), B = (0,0,halfLength) \f$.
class HPP_FCL_DLLAPI Capsule : public ShapeBase {
 public:
  /// @brief Default constructor
  Capsule() {}

  Capsule(FCL_REAL radius_, FCL_REAL lz_) : ShapeBase(), radius(radius_) {
    halfLength = lz_ / 2;
  }

  Capsule(const Capsule& other)
      : ShapeBase(other), radius(other.radius), halfLength(other.halfLength) {}

  /// @brief Clone *this into a new Capsule
  virtual Capsule* clone() const { return new Capsule(*this); };

  /// @brief Radius of capsule
  FCL_REAL radius;

  /// @brief Half Length along z axis
  FCL_REAL halfLength;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a capsule
  NODE_TYPE getNodeType() const { return GEOM_CAPSULE; }

  FCL_REAL computeVolume() const {
    return boost::math::constants::pi<FCL_REAL>() * radius * radius *
           ((halfLength * 2) + radius * 4 / 3.0);
  }

  Matrix3f computeMomentofInertia() const {
    FCL_REAL v_cyl = radius * radius * (halfLength * 2) *
                     boost::math::constants::pi<FCL_REAL>();
    FCL_REAL v_sph = radius * radius * radius *
                     boost::math::constants::pi<FCL_REAL>() * 4 / 3.0;

    FCL_REAL h2 = halfLength * halfLength;
    FCL_REAL r2 = radius * radius;
    FCL_REAL ix = v_cyl * (h2 / 3. + r2 / 4.) +
                  v_sph * (0.4 * r2 + h2 + 0.75 * radius * halfLength);
    FCL_REAL iz = (0.5 * v_cyl + 0.4 * v_sph) * radius * radius;

    return (Matrix3f() << ix, 0, 0, 0, ix, 0, 0, 0, iz).finished();
  }

  FCL_REAL minInflationValue() const { return -radius; }

  /// \brief Inflate the capsule by an amount given by value
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated capsule and the related transform to account for
  /// the change of shape frame
  std::pair<Capsule, Transform3f> inflated(const FCL_REAL value) const {
    if (value <= minInflationValue())
      HPP_FCL_THROW_PRETTY(
          "value (" << value << ") is two small. It should be at least: "
                    << minInflationValue(),
          std::invalid_argument);
    return std::make_pair(Capsule(radius + value, 2 * halfLength),
                          Transform3f());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Capsule* other_ptr = dynamic_cast<const Capsule*>(&_other);
    if (other_ptr == nullptr) return false;
    const Capsule& other = *other_ptr;

    return radius == other.radius && halfLength == other.halfLength;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Cone
/// The base of the cone is at \f$ z = - halfLength \f$ and the top is at
/// \f$ z = halfLength \f$.
class HPP_FCL_DLLAPI Cone : public ShapeBase {
 public:
  /// @brief Default constructor
  Cone() {}

  Cone(FCL_REAL radius_, FCL_REAL lz_) : ShapeBase(), radius(radius_) {
    halfLength = lz_ / 2;
  }

  Cone(const Cone& other)
      : ShapeBase(other), radius(other.radius), halfLength(other.halfLength) {}

  /// @brief Clone *this into a new Cone
  virtual Cone* clone() const { return new Cone(*this); };

  /// @brief Radius of the cone
  FCL_REAL radius;

  /// @brief Half Length along z axis
  FCL_REAL halfLength;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a cone
  NODE_TYPE getNodeType() const { return GEOM_CONE; }

  FCL_REAL computeVolume() const {
    return boost::math::constants::pi<FCL_REAL>() * radius * radius *
           (halfLength * 2) / 3;
  }

  Matrix3f computeMomentofInertia() const {
    FCL_REAL V = computeVolume();
    FCL_REAL ix =
        V * (0.4 * halfLength * halfLength + 3 * radius * radius / 20);
    FCL_REAL iz = 0.3 * V * radius * radius;

    return (Matrix3f() << ix, 0, 0, 0, ix, 0, 0, 0, iz).finished();
  }

  Vec3f computeCOM() const { return Vec3f(0, 0, -0.5 * halfLength); }

  FCL_REAL minInflationValue() const { return -(std::min)(radius, halfLength); }

  /// \brief Inflate the cone by an amount given by value
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated cone and the related transform to account for the
  /// change of shape frame
  std::pair<Cone, Transform3f> inflated(const FCL_REAL value) const {
    if (value <= minInflationValue())
      HPP_FCL_THROW_PRETTY(
          "value (" << value << ") is two small. It should be at least: "
                    << minInflationValue(),
          std::invalid_argument);

    // tan(alpha) = 2*halfLength/radius;
    const FCL_REAL tan_alpha = 2 * halfLength / radius;
    const FCL_REAL sin_alpha = tan_alpha / std::sqrt(1 + tan_alpha * tan_alpha);
    const FCL_REAL top_inflation = value / sin_alpha;
    const FCL_REAL bottom_inflation = value;

    const FCL_REAL new_lz = 2 * halfLength + top_inflation + bottom_inflation;
    const FCL_REAL new_cz = (top_inflation + bottom_inflation) / 2.;
    const FCL_REAL new_radius = new_lz / tan_alpha;

    return std::make_pair(Cone(new_radius, new_lz),
                          Transform3f(Vec3f(0., 0., new_cz)));
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Cone* other_ptr = dynamic_cast<const Cone*>(&_other);
    if (other_ptr == nullptr) return false;
    const Cone& other = *other_ptr;

    return radius == other.radius && halfLength == other.halfLength;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Cylinder along Z axis.
/// The cylinder is defined at its centroid.
class HPP_FCL_DLLAPI Cylinder : public ShapeBase {
 public:
  /// @brief Default constructor
  Cylinder() {}

  Cylinder(FCL_REAL radius_, FCL_REAL lz_) : ShapeBase(), radius(radius_) {
    halfLength = lz_ / 2;
  }

  Cylinder(const Cylinder& other)
      : ShapeBase(other), radius(other.radius), halfLength(other.halfLength) {}

  Cylinder& operator=(const Cylinder& other) {
    if (this == &other) return *this;

    this->radius = other.radius;
    this->halfLength = other.halfLength;
    return *this;
  }

  /// @brief Clone *this into a new Cylinder
  virtual Cylinder* clone() const { return new Cylinder(*this); };

  /// @brief Radius of the cylinder
  FCL_REAL radius;

  /// @brief Half Length along z axis
  FCL_REAL halfLength;

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a cylinder
  NODE_TYPE getNodeType() const { return GEOM_CYLINDER; }

  FCL_REAL computeVolume() const {
    return boost::math::constants::pi<FCL_REAL>() * radius * radius *
           (halfLength * 2);
  }

  Matrix3f computeMomentofInertia() const {
    FCL_REAL V = computeVolume();
    FCL_REAL ix = V * (radius * radius / 4 + halfLength * halfLength / 3);
    FCL_REAL iz = V * radius * radius / 2;
    return (Matrix3f() << ix, 0, 0, 0, ix, 0, 0, 0, iz).finished();
  }

  FCL_REAL minInflationValue() const { return -(std::min)(radius, halfLength); }

  /// \brief Inflate the cylinder by an amount given by value
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated cylinder and the related transform to account for
  /// the change of shape frame
  std::pair<Cylinder, Transform3f> inflated(const FCL_REAL value) const {
    if (value <= minInflationValue())
      HPP_FCL_THROW_PRETTY(
          "value (" << value << ") is two small. It should be at least: "
                    << minInflationValue(),
          std::invalid_argument);
    return std::make_pair(Cylinder(radius + value, 2 * (halfLength + value)),
                          Transform3f());
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Cylinder* other_ptr = dynamic_cast<const Cylinder*>(&_other);
    if (other_ptr == nullptr) return false;
    const Cylinder& other = *other_ptr;

    return radius == other.radius && halfLength == other.halfLength;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Base for convex polytope.
/// @note Inherited classes are responsible for filling ConvexBase::neighbors;
class HPP_FCL_DLLAPI ConvexBase : public ShapeBase {
 public:
  /// @brief Build a convex hull based on Qhull library
  /// and store the vertices and optionally the triangles
  /// \param points, num_points the points whose convex hull should be computed.
  /// \param keepTriangles if \c true, returns a Convex<Triangle> object which
  ///        contains the triangle of the shape.
  /// \param qhullCommand the command sent to qhull.
  ///        - if \c keepTriangles is \c true, this parameter should include
  ///          "Qt". If \c NULL, "Qt" is passed to Qhull.
  ///        - if \c keepTriangles is \c false, an empty string is passed to
  ///          Qhull.
  /// \note hpp-fcl must have been compiled with option \c HPP_FCL_HAS_QHULL set
  ///       to \c ON.
  static ConvexBase* convexHull(const Vec3f* points, unsigned int num_points,
                                bool keepTriangles,
                                const char* qhullCommand = NULL);

  virtual ~ConvexBase();

  ///  @brief Clone (deep copy).
  virtual ConvexBase* clone() const {
    ConvexBase* copy_ptr = new ConvexBase(*this);
    ConvexBase& copy = *copy_ptr;

    if (!copy.own_storage_) {
      copy.points = new Vec3f[copy.num_points];
      std::copy(points, points + num_points, copy.points);
    }
    copy.own_storage_ = true;
    copy.ShapeBase::operator=(*this);

    return copy_ptr;
  }

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a conex polytope
  NODE_TYPE getNodeType() const { return GEOM_CONVEX; }

  /// @brief An array of the points of the polygon.
  Vec3f* points;
  unsigned int num_points;

  struct HPP_FCL_DLLAPI Neighbors {
    unsigned char count_;
    unsigned int* n_;

    unsigned char const& count() const { return count_; }
    unsigned int& operator[](int i) {
      assert(i < count_);
      return n_[i];
    }
    unsigned int const& operator[](int i) const {
      assert(i < count_);
      return n_[i];
    }

    bool operator==(const Neighbors& other) const {
      if (count_ != other.count_) return false;

      for (int i = 0; i < count_; ++i) {
        if (n_[i] != other.n_[i]) return false;
      }

      return true;
    }

    bool operator!=(const Neighbors& other) const { return !(*this == other); }
  };
  /// Neighbors of each vertex.
  /// It is an array of size num_points. For each vertex, it contains the number
  /// of neighbors and a list of indices to them.
  Neighbors* neighbors;

  /// @brief center of the convex polytope, this is used for collision: center
  /// is guaranteed in the internal of the polytope (as it is convex)
  Vec3f center;

 protected:
  /// @brief Construct an uninitialized convex object
  /// Initialization is done with ConvexBase::initialize.
  ConvexBase()
      : ShapeBase(),
        points(NULL),
        num_points(0),
        neighbors(NULL),
        nneighbors_(NULL),
        own_storage_(false) {}

  /// @brief Initialize the points of the convex shape
  /// This also initializes the ConvexBase::center.
  ///
  /// \param ownStorage weither the ConvexBase owns the data.
  /// \param points_ list of 3D points  ///
  /// \param num_points_ number of 3D points
  void initialize(bool ownStorage, Vec3f* points_, unsigned int num_points_);

  /// @brief Set the points of the convex shape.
  ///
  /// \param ownStorage weither the ConvexBase owns the data.
  /// \param points_ list of 3D points  ///
  /// \param num_points_ number of 3D points
  void set(bool ownStorage, Vec3f* points_, unsigned int num_points_);

  /// @brief Copy constructor
  /// Only the list of neighbors is copied.
  ConvexBase(const ConvexBase& other);

  unsigned int* nneighbors_;

  bool own_storage_;

 private:
  void computeCenter();

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const ConvexBase* other_ptr = dynamic_cast<const ConvexBase*>(&_other);
    if (other_ptr == nullptr) return false;
    const ConvexBase& other = *other_ptr;

    if (num_points != other.num_points) return false;

    for (unsigned int i = 0; i < num_points; ++i) {
      if (points[i] != other.points[i]) return false;
    }

    for (unsigned int i = 0; i < num_points; ++i) {
      if (neighbors[i] != other.neighbors[i]) return false;
    }

    return center == other.center;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PolygonT>
class Convex;

/// @brief Half Space: this is equivalent to the Plane in ODE. The separation
/// plane is defined as n * x = d; Points in the negative side of the separation
/// plane (i.e. {x | n * x < d}) are inside the half space and points in the
/// positive side of the separation plane (i.e. {x | n * x > d}) are outside the
/// half space
class HPP_FCL_DLLAPI Halfspace : public ShapeBase {
 public:
  /// @brief Construct a half space with normal direction and offset
  Halfspace(const Vec3f& n_, FCL_REAL d_) : ShapeBase(), n(n_), d(d_) {
    unitNormalTest();
  }

  /// @brief Construct a plane with normal direction and offset
  Halfspace(FCL_REAL a, FCL_REAL b, FCL_REAL c, FCL_REAL d_)
      : ShapeBase(), n(a, b, c), d(d_) {
    unitNormalTest();
  }

  Halfspace() : ShapeBase(), n(1, 0, 0), d(0) {}

  Halfspace(const Halfspace& other)
      : ShapeBase(other), n(other.n), d(other.d) {}

  /// @brief operator =
  Halfspace& operator=(const Halfspace& other) {
    n = other.n;
    d = other.d;
    return *this;
  }

  /// @brief Clone *this into a new Halfspace
  virtual Halfspace* clone() const { return new Halfspace(*this); };

  FCL_REAL signedDistance(const Vec3f& p) const { return n.dot(p) - d; }

  FCL_REAL distance(const Vec3f& p) const { return std::abs(n.dot(p) - d); }

  /// @brief Compute AABB
  void computeLocalAABB();

  /// @brief Get node type: a half space
  NODE_TYPE getNodeType() const { return GEOM_HALFSPACE; }

  FCL_REAL minInflationValue() const {
    return std::numeric_limits<FCL_REAL>::lowest();
  }

  /// \brief Inflate the cylinder by an amount given by value
  ///
  /// \param[in] value of the shape inflation.
  ///
  /// \returns a new inflated cylinder and the related transform to account for
  /// the change of shape frame
  std::pair<Halfspace, Transform3f> inflated(const FCL_REAL value) const {
    if (value <= minInflationValue())
      HPP_FCL_THROW_PRETTY(
          "value (" << value << ") is two small. It should be at least: "
                    << minInflationValue(),
          std::invalid_argument);
    return std::make_pair(Halfspace(n, d + value), Transform3f());
  }

  /// @brief Plane normal
  Vec3f n;

  /// @brief Plane offset
  FCL_REAL d;

 protected:
  /// @brief Turn non-unit normal into unit
  void unitNormalTest();

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Halfspace* other_ptr = dynamic_cast<const Halfspace*>(&_other);
    if (other_ptr == nullptr) return false;
    const Halfspace& other = *other_ptr;

    return n == other.n && d == other.d;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @brief Infinite plane
class HPP_FCL_DLLAPI Plane : public ShapeBase {
 public:
  /// @brief Construct a plane with normal direction and offset
  Plane(const Vec3f& n_, FCL_REAL d_) : ShapeBase(), n(n_), d(d_) {
    unitNormalTest();
  }

  /// @brief Construct a plane with normal direction and offset
  Plane(FCL_REAL a, FCL_REAL b, FCL_REAL c, FCL_REAL d_)
      : ShapeBase(), n(a, b, c), d(d_) {
    unitNormalTest();
  }

  Plane() : ShapeBase(), n(1, 0, 0), d(0) {}

  Plane(const Plane& other) : ShapeBase(other), n(other.n), d(other.d) {}

  /// @brief operator =
  Plane& operator=(const Plane& other) {
    n = other.n;
    d = other.d;
    return *this;
  }

  /// @brief Clone *this into a new Plane
  virtual Plane* clone() const { return new Plane(*this); };

  FCL_REAL signedDistance(const Vec3f& p) const { return n.dot(p) - d; }

  FCL_REAL distance(const Vec3f& p) const { return std::abs(n.dot(p) - d); }

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

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const Plane* other_ptr = dynamic_cast<const Plane*>(&_other);
    if (other_ptr == nullptr) return false;
    const Plane& other = *other_ptr;

    return n == other.n && d == other.d;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fcl

}  // namespace hpp

#endif
