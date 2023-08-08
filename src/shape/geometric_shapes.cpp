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

void ConvexBase::initialize(std::shared_ptr<Vec3f> points_,
                            unsigned int num_points_) {
  points = points_;
  num_points = num_points_;
  computeCenter();
}

void ConvexBase::set(std::shared_ptr<Vec3f> points_, unsigned int num_points_) {
  initialize(points_, num_points_);
}

ConvexBase::ConvexBase(const ConvexBase& other)
    : ShapeBase(other), num_points(other.num_points), center(other.center) {
  if (other.points.get()) {
    points.reset(new Vec3f[num_points]);
    std::copy(other.points.get(), other.points.get() + num_points,
              points.get());
  } else
    points.reset();

  if (other.neighbors.get()) {
    neighbors.reset(new Neighbors[num_points]);
    std::copy(other.neighbors.get(), other.neighbors.get() + num_points,
              neighbors.get());
  } else
    neighbors.reset();

  if (other.nneighbors_.get()) {
    std::size_t c_nneighbors = 0;
    const Neighbors* neighbors_ = neighbors.get();
    for (std::size_t i = 0; i < num_points; ++i)
      c_nneighbors += neighbors_[i].count();
    nneighbors_.reset(new unsigned int[c_nneighbors]);
    std::copy(other.nneighbors_.get(), other.nneighbors_.get() + c_nneighbors,
              nneighbors_.get());
  } else
    nneighbors_.reset();

  ShapeBase::operator=(*this);
}

ConvexBase::~ConvexBase() {}

void ConvexBase::computeCenter() {
  center.setZero();
  const Vec3f* points_ = points.get();
  for (std::size_t i = 0; i < num_points; ++i)
    center += points_[i];  // TODO(jcarpent): vectorization
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
