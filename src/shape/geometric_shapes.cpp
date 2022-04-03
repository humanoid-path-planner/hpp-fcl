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

#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/shape/geometric_shapes_utility.h>

namespace hpp {
namespace fcl {

void ConvexBase::initialize(bool own_storage, Vec3f* points_,
                            unsigned int num_points_) {
  points = points_;
  num_points = num_points_;
  own_storage_ = own_storage;
  computeCenter();
}

void ConvexBase::set(bool own_storage_, Vec3f* points_,
                     unsigned int num_points_) {
  if (own_storage_ && points) delete[] points;
  initialize(own_storage_, points_, num_points_);
}

ConvexBase::ConvexBase(const ConvexBase& other)
    : ShapeBase(other),
      num_points(other.num_points),
      center(other.center),
      own_storage_(other.own_storage_) {
  if (neighbors) delete[] neighbors;
  if (nneighbors_) delete[] nneighbors_;
  if (own_storage_) {
    if (own_storage_ && points) delete[] points;

    points = new Vec3f[num_points];
    std::copy(other.points, other.points + num_points, points);
  } else
    points = other.points;

  neighbors = new Neighbors[num_points];
  std::copy(other.neighbors, other.neighbors + num_points, neighbors);

  std::size_t c_nneighbors = 0;
  for (std::size_t i = 0; i < num_points; ++i)
    c_nneighbors += neighbors[i].count();
  nneighbors_ = new unsigned int[c_nneighbors];
  std::copy(other.nneighbors_, other.nneighbors_ + c_nneighbors, nneighbors_);

  ShapeBase::operator=(*this);
}

ConvexBase::~ConvexBase() {
  if (neighbors) delete[] neighbors;
  if (nneighbors_) delete[] nneighbors_;
  if (own_storage_ && points) delete[] points;
}

void ConvexBase::computeCenter() {
  center.setZero();
  for (std::size_t i = 0; i < num_points; ++i)
    center += points[i];  // TODO(jcarpent): vectorization
  center /= num_points;
}

void Halfspace::unitNormalTest() {
  FCL_REAL l = n.norm();
  if (l > 0) {
    FCL_REAL inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  } else {
    n << 1, 0, 0;
    d = 0;
  }
}

void Plane::unitNormalTest() {
  FCL_REAL l = n.norm();
  if (l > 0) {
    FCL_REAL inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  } else {
    n << 1, 0, 0;
    d = 0;
  }
}

void Box::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Sphere::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = radius;
}

void Ellipsoid::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Capsule::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Cone::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Cylinder::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void ConvexBase::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Halfspace::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Plane::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void TriangleP::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

}  // namespace fcl

}  // namespace hpp
