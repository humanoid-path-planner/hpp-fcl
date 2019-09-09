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

#include <set>

namespace hpp
{
namespace fcl
{

void Convex::initialize()
{
  center.setZero();
  for(int i = 0; i < num_points; ++i)
    center += points[i];
  center /= num_points;

  int *points_in_poly = polygons;
  neighbors = new Neighbors[num_points];

  std::vector<std::set<unsigned int> > nneighbors (num_points);
  unsigned int c_nneighbors = 0;

  for (int l = 0; l < num_polygons; ++l)
  {
    int n = *points_in_poly;

    for (int j = 0; j < n; ++j) {
      int i = (j==0  ) ? n-1 : j-1;
      int k = (j==n-1) ? 0   : j+1;
      int pi = points_in_poly[i+1];
      int pj = points_in_poly[j+1];
      int pk = points_in_poly[k+1];
      // Update neighbors of pj;
      if (nneighbors[pj].count(pi) == 0) {
        c_nneighbors++;
        nneighbors[pj].insert(pi);
      }
      if (nneighbors[pj].count(pk) == 0) {
        c_nneighbors++;
        nneighbors[pj].insert(pk);
      }
    }

    points_in_poly += n+1;
  }

  nneighbors_ = new unsigned int[c_nneighbors];
  unsigned int* p_nneighbors = nneighbors_;
  for (int i = 0; i < num_points; ++i) {
    Neighbors& n = neighbors[i];
    if (nneighbors[i].size() >= std::numeric_limits<unsigned char>::max())
      throw std::logic_error ("Too many neighbors.");
    n.count_ = (unsigned char)nneighbors[i].size();
    n.n_     = p_nneighbors;
    p_nneighbors = std::copy (nneighbors[i].begin(), nneighbors[i].end(), p_nneighbors);
  }
  assert (p_nneighbors == nneighbors_ + c_nneighbors);
}

void Halfspace::unitNormalTest()
{
  FCL_REAL l = n.norm();
  if(l > 0)
  {
    FCL_REAL inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  }
  else
  {
    n << 1, 0, 0;
    d = 0;
  }  
}

void Plane::unitNormalTest()
{
  FCL_REAL l = n.norm();
  if(l > 0)
  {
    FCL_REAL inv_l = 1.0 / l;
    n *= inv_l;
    d *= inv_l;
  }
  else
  {
    n << 1, 0, 0;
    d = 0;
  }
}


void Box::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Sphere::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = radius;
}

void Capsule::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Cone::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Cylinder::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Convex::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Halfspace::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Plane::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void TriangleP::computeLocalAABB()
{
  computeBV<AABB>(*this, Transform3f(), aabb_local);
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}


}

} // namespace hpp
