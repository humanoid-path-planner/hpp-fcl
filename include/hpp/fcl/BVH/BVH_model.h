/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2020-2022, INRIA
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

#ifndef HPP_FCL_BVH_MODEL_H
#define HPP_FCL_BVH_MODEL_H

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/BVH/BVH_internal.h>
#include <hpp/fcl/BV/BV_node.h>
#include <vector>

namespace hpp {
namespace fcl {

/// @addtogroup Construction_Of_BVH
/// @{

class ConvexBase;

template <typename BV>
class BVFitter;
template <typename BV>
class BVSplitter;

/// @brief A base class describing the bounding hierarchy of a mesh model or a
/// point cloud model (which is viewed as a degraded version of mesh)
class HPP_FCL_DLLAPI BVHModelBase : public CollisionGeometry {
 public:
  /// @brief Geometry point data
  Vec3f* vertices;

  /// @brief Geometry triangle index data, will be NULL for point clouds
  Triangle* tri_indices;

  /// @brief Geometry point data in previous frame
  Vec3f* prev_vertices;

  /// @brief Number of triangles
  unsigned int num_tris;

  /// @brief Number of points
  unsigned int num_vertices;

  /// @brief The state of BVH building process
  BVHBuildState build_state;

  /// @brief Convex<Triangle> representation of this object
  shared_ptr<ConvexBase> convex;

  /// @brief Model type described by the instance
  BVHModelType getModelType() const {
    if (num_tris && num_vertices)
      return BVH_MODEL_TRIANGLES;
    else if (num_vertices)
      return BVH_MODEL_POINTCLOUD;
    else
      return BVH_MODEL_UNKNOWN;
  }

  /// @brief Constructing an empty BVH
  BVHModelBase();

  /// @brief copy from another BVH
  BVHModelBase(const BVHModelBase& other);

  /// @brief deconstruction, delete mesh data related.
  virtual ~BVHModelBase() {
    delete[] vertices;
    delete[] tri_indices;
    delete[] prev_vertices;
  }

  /// @brief Get the object type: it is a BVH
  OBJECT_TYPE getObjectType() const { return OT_BVH; }

  /// @brief Compute the AABB for the BVH, used for broad-phase collision
  void computeLocalAABB();

  /// @brief Begin a new BVH model
  int beginModel(unsigned int num_tris = 0, unsigned int num_vertices = 0);

  /// @brief Add one point in the new BVH model
  int addVertex(const Vec3f& p);

  /// @brief Add points in the new BVH model
  int addVertices(const Matrixx3f& points);

  /// @brief Add triangles in the new BVH model
  int addTriangles(const Matrixx3i& triangles);

  /// @brief Add one triangle in the new BVH model
  int addTriangle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3);

  /// @brief Add a set of triangles in the new BVH model
  int addSubModel(const std::vector<Vec3f>& ps,
                  const std::vector<Triangle>& ts);

  /// @brief Add a set of points in the new BVH model
  int addSubModel(const std::vector<Vec3f>& ps);

  /// @brief End BVH model construction, will build the bounding volume
  /// hierarchy
  int endModel();

  /// @brief Replace the geometry information of current frame (i.e. should have
  /// the same mesh topology with the previous frame)
  int beginReplaceModel();

  /// @brief Replace one point in the old BVH model
  int replaceVertex(const Vec3f& p);

  /// @brief Replace one triangle in the old BVH model
  int replaceTriangle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3);

  /// @brief Replace a set of points in the old BVH model
  int replaceSubModel(const std::vector<Vec3f>& ps);

  /// @brief End BVH model replacement, will also refit or rebuild the bounding
  /// volume hierarchy
  int endReplaceModel(bool refit = true, bool bottomup = true);

  /// @brief Replace the geometry information of current frame (i.e. should have
  /// the same mesh topology with the previous frame). The current frame will be
  /// saved as the previous frame in prev_vertices.
  int beginUpdateModel();

  /// @brief Update one point in the old BVH model
  int updateVertex(const Vec3f& p);

  /// @brief Update one triangle in the old BVH model
  int updateTriangle(const Vec3f& p1, const Vec3f& p2, const Vec3f& p3);

  /// @brief Update a set of points in the old BVH model
  int updateSubModel(const std::vector<Vec3f>& ps);

  /// @brief End BVH model update, will also refit or rebuild the bounding
  /// volume hierarchy
  int endUpdateModel(bool refit = true, bool bottomup = true);

  /// @brief Build this \ref Convex "Convex<Triangle>" representation of this
  /// model. The result is stored in attribute \ref convex. \note this only
  /// takes the points of this model. It does not check that the
  ///       object is convex. It does not compute a convex hull.
  void buildConvexRepresentation(bool share_memory);

  /// @brief Build a convex hull
  /// and store it in attribute \ref convex.
  /// \param keepTriangle whether the convex should be triangulated.
  /// \param qhullCommand see \ref ConvexBase::convexHull.
  /// \return \c true if this object is convex, hence the convex hull represents
  ///         the same object.
  /// \sa ConvexBase::convexHull
  /// \warning At the moment, the return value only checks whether there are as
  ///          many points in the convex hull as in the original object. This is
  ///          neither necessary (duplicated vertices get merged) nor sufficient
  ///          (think of a U with 4 vertices and 3 edges).
  bool buildConvexHull(bool keepTriangle, const char* qhullCommand = NULL);

  virtual int memUsage(const bool msg = false) const = 0;

  /// @brief This is a special acceleration: BVH_model default stores the BV's
  /// transform in world coordinate. However, we can also store each BV's
  /// transform related to its parent BV node. When traversing the BVH, this can
  /// save one matrix transformation.
  virtual void makeParentRelative() = 0;

  Vec3f computeCOM() const {
    FCL_REAL vol = 0;
    Vec3f com(0, 0, 0);
    for (unsigned int i = 0; i < num_tris; ++i) {
      const Triangle& tri = tri_indices[i];
      FCL_REAL d_six_vol =
          (vertices[tri[0]].cross(vertices[tri[1]])).dot(vertices[tri[2]]);
      vol += d_six_vol;
      com +=
          (vertices[tri[0]] + vertices[tri[1]] + vertices[tri[2]]) * d_six_vol;
    }

    return com / (vol * 4);
  }

  FCL_REAL computeVolume() const {
    FCL_REAL vol = 0;
    for (unsigned int i = 0; i < num_tris; ++i) {
      const Triangle& tri = tri_indices[i];
      FCL_REAL d_six_vol =
          (vertices[tri[0]].cross(vertices[tri[1]])).dot(vertices[tri[2]]);
      vol += d_six_vol;
    }

    return vol / 6;
  }

  Matrix3f computeMomentofInertia() const {
    Matrix3f C = Matrix3f::Zero();

    Matrix3f C_canonical;
    C_canonical << 1 / 60.0, 1 / 120.0, 1 / 120.0, 1 / 120.0, 1 / 60.0,
        1 / 120.0, 1 / 120.0, 1 / 120.0, 1 / 60.0;

    for (unsigned int i = 0; i < num_tris; ++i) {
      const Triangle& tri = tri_indices[i];
      const Vec3f& v1 = vertices[tri[0]];
      const Vec3f& v2 = vertices[tri[1]];
      const Vec3f& v3 = vertices[tri[2]];
      Matrix3f A;
      A << v1.transpose(), v2.transpose(), v3.transpose();
      C += A.derived().transpose() * C_canonical * A * (v1.cross(v2)).dot(v3);
    }

    return C.trace() * Matrix3f::Identity() - C;
  }

 protected:
  virtual void deleteBVs() = 0;
  virtual bool allocateBVs() = 0;

  /// @brief Build the bounding volume hierarchy
  virtual int buildTree() = 0;

  /// @brief Refit the bounding volume hierarchy
  virtual int refitTree(bool bottomup) = 0;

  unsigned int num_tris_allocated;
  unsigned int num_vertices_allocated;
  unsigned int num_vertex_updated;  /// for ccd vertex update

 protected:
  /// \brief Comparison operators
  virtual bool isEqual(const CollisionGeometry& other) const;
};

/// @brief A class describing the bounding hierarchy of a mesh model or a point
/// cloud model (which is viewed as a degraded version of mesh) \tparam BV one
/// of the bounding volume class in \ref Bounding_Volume.
template <typename BV>
class HPP_FCL_DLLAPI BVHModel : public BVHModelBase {
  typedef BVHModelBase Base;

 public:
  /// @brief Split rule to split one BV node into two children
  shared_ptr<BVSplitter<BV> > bv_splitter;

  /// @brief Fitting rule to fit a BV node to a set of geometry primitives
  shared_ptr<BVFitter<BV> > bv_fitter;

  /// @brief Default constructor to build an empty BVH
  BVHModel();

  /// @brief Copy constructor from another BVH
  ///
  /// \param[in] other BVHModel to copy.
  ///
  BVHModel(const BVHModel& other);

  /// @brief Clone *this into a new BVHModel
  virtual BVHModel<BV>* clone() const { return new BVHModel(*this); }

  /// @brief deconstruction, delete mesh data related.
  ~BVHModel() {
    delete[] bvs;
    delete[] primitive_indices;
  }

  /// @brief We provide getBV() and getNumBVs() because BVH may be compressed
  /// (in future), so we must provide some flexibility here

  /// @brief Access the bv giving the its index
  const BVNode<BV>& getBV(unsigned int i) const {
    assert(i < num_bvs);
    return bvs[i];
  }

  /// @brief Access the bv giving the its index
  BVNode<BV>& getBV(unsigned int i) {
    assert(i < num_bvs);
    return bvs[i];
  }

  /// @brief Get the number of bv in the BVH
  unsigned int getNumBVs() const { return num_bvs; }

  /// @brief Get the BV type: default is unknown
  NODE_TYPE getNodeType() const { return BV_UNKNOWN; }

  /// @brief Check the number of memory used
  int memUsage(const bool msg) const;

  /// @brief This is a special acceleration: BVH_model default stores the BV's
  /// transform in world coordinate. However, we can also store each BV's
  /// transform related to its parent BV node. When traversing the BVH, this can
  /// save one matrix transformation.
  void makeParentRelative() {
    Matrix3f I(Matrix3f::Identity());
    makeParentRelativeRecurse(0, I, Vec3f::Zero());
  }

 protected:
  void deleteBVs();
  bool allocateBVs();

  unsigned int num_bvs_allocated;
  unsigned int* primitive_indices;

  /// @brief Bounding volume hierarchy
  BVNode<BV>* bvs;

  /// @brief Number of BV nodes in bounding volume hierarchy
  unsigned int num_bvs;

  /// @brief Build the bounding volume hierarchy
  int buildTree();

  /// @brief Refit the bounding volume hierarchy
  int refitTree(bool bottomup);

  /// @brief Refit the bounding volume hierarchy in a top-down way (slow but
  /// more compact)
  int refitTree_topdown();

  /// @brief Refit the bounding volume hierarchy in a bottom-up way (fast but
  /// less compact)
  int refitTree_bottomup();

  /// @brief Recursive kernel for hierarchy construction
  int recursiveBuildTree(int bv_id, unsigned int first_primitive,
                         unsigned int num_primitives);

  /// @brief Recursive kernel for bottomup refitting
  int recursiveRefitTree_bottomup(int bv_id);

  /// @ recursively compute each bv's transform related to its parent. For
  /// default BV, only the translation works. For oriented BV (OBB, RSS,
  /// OBBRSS), special implementation is provided.
  void makeParentRelativeRecurse(int bv_id, Matrix3f& parent_axes,
                                 const Vec3f& parent_c) {
    if (!bvs[bv_id].isLeaf()) {
      makeParentRelativeRecurse(bvs[bv_id].first_child, parent_axes,
                                bvs[bv_id].getCenter());

      makeParentRelativeRecurse(bvs[bv_id].first_child + 1, parent_axes,
                                bvs[bv_id].getCenter());
    }

    bvs[bv_id].bv = translate(bvs[bv_id].bv, -parent_c);
  }

 private:
  virtual bool isEqual(const CollisionGeometry& _other) const {
    const BVHModel* other_ptr = dynamic_cast<const BVHModel*>(&_other);
    if (other_ptr == nullptr) return false;
    const BVHModel& other = *other_ptr;

    bool res = Base::isEqual(other);
    if (!res) return false;

    // unsigned int other_num_primitives = 0;
    // if(other.primitive_indices)
    // {

    //   switch(other.getModelType())
    //   {
    //     case BVH_MODEL_TRIANGLES:
    //       other_num_primitives = num_tris;
    //       break;
    //     case BVH_MODEL_POINTCLOUD:
    //       other_num_primitives = num_vertices;
    //       break;
    //     default:
    //       ;
    //   }
    // }

    //    unsigned int num_primitives = 0;
    //    if(primitive_indices)
    //    {
    //
    //      switch(other.getModelType())
    //      {
    //        case BVH_MODEL_TRIANGLES:
    //          num_primitives = num_tris;
    //          break;
    //        case BVH_MODEL_POINTCLOUD:
    //          num_primitives = num_vertices;
    //          break;
    //        default:
    //          ;
    //      }
    //    }
    //
    //    if(num_primitives != other_num_primitives)
    //      return false;
    //
    //    for(int k = 0; k < num_primitives; ++k)
    //    {
    //      if(primitive_indices[k] != other.primitive_indices[k])
    //        return false;
    //    }

    if (num_bvs != other.num_bvs) return false;

    for (unsigned int k = 0; k < num_bvs; ++k) {
      if (bvs[k] != other.bvs[k]) return false;
    }

    return true;
  }
};

/// @}

template <>
void BVHModel<OBB>::makeParentRelativeRecurse(int bv_id, Matrix3f& parent_axes,
                                              const Vec3f& parent_c);

template <>
void BVHModel<RSS>::makeParentRelativeRecurse(int bv_id, Matrix3f& parent_axes,
                                              const Vec3f& parent_c);

template <>
void BVHModel<OBBRSS>::makeParentRelativeRecurse(int bv_id,
                                                 Matrix3f& parent_axes,
                                                 const Vec3f& parent_c);

/// @brief Specialization of getNodeType() for BVHModel with different BV types
template <>
NODE_TYPE BVHModel<AABB>::getNodeType() const;

template <>
NODE_TYPE BVHModel<OBB>::getNodeType() const;

template <>
NODE_TYPE BVHModel<RSS>::getNodeType() const;

template <>
NODE_TYPE BVHModel<kIOS>::getNodeType() const;

template <>
NODE_TYPE BVHModel<OBBRSS>::getNodeType() const;

template <>
NODE_TYPE BVHModel<KDOP<16> >::getNodeType() const;

template <>
NODE_TYPE BVHModel<KDOP<18> >::getNodeType() const;

template <>
NODE_TYPE BVHModel<KDOP<24> >::getNodeType() const;

}  // namespace fcl

}  // namespace hpp

#endif
