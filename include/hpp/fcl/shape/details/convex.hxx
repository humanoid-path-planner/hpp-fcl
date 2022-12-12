/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, CNRS - LAAS
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

/** \author Joseph Mirabel */

#ifndef HPP_FCL_SHAPE_CONVEX_HXX
#define HPP_FCL_SHAPE_CONVEX_HXX

#include <set>
#include <vector>

namespace hpp {
namespace fcl {

template <typename PolygonT>
Convex<PolygonT>::Convex(bool own_storage, Vec3f* points_,
                         unsigned int num_points_, PolygonT* polygons_,
                         unsigned int num_polygons_)
    : ConvexBase(), polygons(polygons_), num_polygons(num_polygons_) {
  initialize(own_storage, points_, num_points_);
  fillNeighbors();
}

template <typename PolygonT>
Convex<PolygonT>::Convex(const Convex<PolygonT>& other)
    : ConvexBase(other),
      polygons(other.polygons),
      num_polygons(other.num_polygons) {
  if (own_storage_) {
    polygons = new PolygonT[num_polygons];
    std::copy(other.polygons, other.polygons + num_polygons, polygons);
  }
}

template <typename PolygonT>
Convex<PolygonT>::~Convex() {
  if (own_storage_) delete[] polygons;
}

template <typename PolygonT>
void Convex<PolygonT>::set(bool own_storage, Vec3f* points_,
                           unsigned int num_points_, PolygonT* polygons_,
                           unsigned int num_polygons_) {
  if (own_storage_) delete[] polygons;
  ConvexBase::set(own_storage, points_, num_points_);

  num_polygons = num_polygons_;
  polygons = polygons_;

  fillNeighbors();
}

template <typename PolygonT>
Convex<PolygonT>* Convex<PolygonT>::clone() const {
  Vec3f* cloned_points = new Vec3f[num_points];
  std::copy(points, points + num_points, cloned_points);

  PolygonT* cloned_polygons = new PolygonT[num_polygons];
  std::copy(polygons, polygons + num_polygons, cloned_polygons);

  Convex* copy_ptr = new Convex(true, cloned_points, num_points,
                                cloned_polygons, num_polygons);

  copy_ptr->ShapeBase::operator=(*this);
  return copy_ptr;
}

template <typename PolygonT>
Matrix3f Convex<PolygonT>::computeMomentofInertia() const {
  typedef typename PolygonT::size_type size_type;
  typedef typename PolygonT::index_type index_type;

  Matrix3f C = Matrix3f::Zero();

  Matrix3f C_canonical;
  C_canonical << 1 / 60.0, 1 / 120.0, 1 / 120.0, 1 / 120.0, 1 / 60.0, 1 / 120.0,
      1 / 120.0, 1 / 120.0, 1 / 60.0;

  for (unsigned int i = 0; i < num_polygons; ++i) {
    const PolygonT& polygon = polygons[i];

    // compute the center of the polygon
    Vec3f plane_center(0, 0, 0);
    for (size_type j = 0; j < polygon.size(); ++j)
      plane_center += points[polygon[(index_type)j]];
    plane_center /= polygon.size();

    // compute the volume of tetrahedron making by neighboring two points, the
    // plane center and the reference point (zero) of the convex shape
    const Vec3f& v3 = plane_center;
    for (size_type j = 0; j < polygon.size(); ++j) {
      index_type e_first = polygon[static_cast<index_type>(j)];
      index_type e_second =
          polygon[static_cast<index_type>((j + 1) % polygon.size())];
      const Vec3f& v1 = points[e_first];
      const Vec3f& v2 = points[e_second];
      Matrix3f A;
      A << v1.transpose(), v2.transpose(),
          v3.transpose();  // this is A' in the original document
      C += A.transpose() * C_canonical * A * (v1.cross(v2)).dot(v3);
    }
  }

  return C.trace() * Matrix3f::Identity() - C;
}

template <typename PolygonT>
Vec3f Convex<PolygonT>::computeCOM() const {
  typedef typename PolygonT::size_type size_type;
  typedef typename PolygonT::index_type index_type;

  Vec3f com(0, 0, 0);
  FCL_REAL vol = 0;
  for (unsigned int i = 0; i < num_polygons; ++i) {
    const PolygonT& polygon = polygons[i];
    // compute the center of the polygon
    Vec3f plane_center(0, 0, 0);
    for (size_type j = 0; j < polygon.size(); ++j)
      plane_center += points[polygon[(index_type)j]];
    plane_center /= polygon.size();

    // compute the volume of tetrahedron making by neighboring two points, the
    // plane center and the reference point (zero) of the convex shape
    const Vec3f& v3 = plane_center;
    for (size_type j = 0; j < polygon.size(); ++j) {
      index_type e_first = polygon[static_cast<index_type>(j)];
      index_type e_second =
          polygon[static_cast<index_type>((j + 1) % polygon.size())];
      const Vec3f& v1 = points[e_first];
      const Vec3f& v2 = points[e_second];
      FCL_REAL d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
      com += (points[e_first] + points[e_second] + plane_center) * d_six_vol;
    }
  }

  return com / (vol * 4);  // here we choose zero as the reference
}

template <typename PolygonT>
FCL_REAL Convex<PolygonT>::computeVolume() const {
  typedef typename PolygonT::size_type size_type;
  typedef typename PolygonT::index_type index_type;

  FCL_REAL vol = 0;
  for (unsigned int i = 0; i < num_polygons; ++i) {
    const PolygonT& polygon = polygons[i];

    // compute the center of the polygon
    Vec3f plane_center(0, 0, 0);
    for (size_type j = 0; j < polygon.size(); ++j)
      plane_center += points[polygon[(index_type)j]];
    plane_center /= polygon.size();

    // compute the volume of tetrahedron making by neighboring two points, the
    // plane center and the reference point (zero point) of the convex shape
    const Vec3f& v3 = plane_center;
    for (size_type j = 0; j < polygon.size(); ++j) {
      index_type e_first = polygon[static_cast<index_type>(j)];
      index_type e_second =
          polygon[static_cast<index_type>((j + 1) % polygon.size())];
      const Vec3f& v1 = points[e_first];
      const Vec3f& v2 = points[e_second];
      FCL_REAL d_six_vol = (v1.cross(v2)).dot(v3);
      vol += d_six_vol;
    }
  }

  return vol / 6;
}

template <typename PolygonT>
void Convex<PolygonT>::fillNeighbors() {
  if (neighbors) delete[] neighbors;
  neighbors = new Neighbors[num_points];

  typedef typename PolygonT::size_type size_type;
  typedef typename PolygonT::index_type index_type;
  std::vector<std::set<index_type> > nneighbors(num_points);
  unsigned int c_nneighbors = 0;

  for (unsigned int l = 0; l < num_polygons; ++l) {
    const PolygonT& polygon = polygons[l];
    const size_type n = polygon.size();

    for (size_type j = 0; j < polygon.size(); ++j) {
      size_type i = (j == 0) ? n - 1 : j - 1;
      size_type k = (j == n - 1) ? 0 : j + 1;
      index_type pi = polygon[(index_type)i], pj = polygon[(index_type)j],
                 pk = polygon[(index_type)k];
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
  }

  if (nneighbors_) delete[] nneighbors_;
  nneighbors_ = new unsigned int[c_nneighbors];

  unsigned int* p_nneighbors = nneighbors_;
  for (unsigned int i = 0; i < num_points; ++i) {
    Neighbors& n = neighbors[i];
    if (nneighbors[i].size() >= (std::numeric_limits<unsigned char>::max)())
      HPP_FCL_THROW_PRETTY("Too many neighbors.", std::logic_error);
    n.count_ = (unsigned char)nneighbors[i].size();
    n.n_ = p_nneighbors;
    p_nneighbors =
        std::copy(nneighbors[i].begin(), nneighbors[i].end(), p_nneighbors);
  }
  assert(p_nneighbors == nneighbors_ + c_nneighbors);
}

}  // namespace fcl

}  // namespace hpp

#endif
