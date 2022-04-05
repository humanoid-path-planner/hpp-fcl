/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2020, INRIA
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

#include <hpp/fcl/BVH/BVH_model.h>

#include <iostream>
#include <string.h>

#include <hpp/fcl/BV/BV.h>
#include <hpp/fcl/shape/convex.h>

#include <hpp/fcl/internal/BV_splitter.h>
#include <hpp/fcl/internal/BV_fitter.h>

namespace hpp {
namespace fcl {

BVHModelBase::BVHModelBase()
    : vertices(NULL),
      tri_indices(NULL),
      prev_vertices(NULL),
      num_tris(0),
      num_vertices(0),
      build_state(BVH_BUILD_STATE_EMPTY),
      num_tris_allocated(0),
      num_vertices_allocated(0),
      num_vertex_updated(0) {}

BVHModelBase::BVHModelBase(const BVHModelBase& other)
    : CollisionGeometry(other),
      num_tris(other.num_tris),
      num_vertices(other.num_vertices),
      build_state(other.build_state),
      num_tris_allocated(other.num_tris),
      num_vertices_allocated(other.num_vertices) {
  if (other.vertices) {
    vertices = new Vec3f[num_vertices];
    std::copy(other.vertices, other.vertices + num_vertices, vertices);
  } else
    vertices = nullptr;

  if (other.tri_indices) {
    tri_indices = new Triangle[num_tris];
    std::copy(other.tri_indices, other.tri_indices + num_tris, tri_indices);
  } else
    tri_indices = nullptr;

  if (other.prev_vertices) {
    prev_vertices = new Vec3f[num_vertices];
    std::copy(other.prev_vertices, other.prev_vertices + num_vertices,
              prev_vertices);
  } else
    prev_vertices = nullptr;
}

bool BVHModelBase::isEqual(const CollisionGeometry& _other) const {
  const BVHModelBase* other_ptr = dynamic_cast<const BVHModelBase*>(&_other);
  if (other_ptr == nullptr) return false;
  const BVHModelBase& other = *other_ptr;

  bool result =
      num_tris == other.num_tris && num_vertices == other.num_vertices;

  if (!result) return false;

  for (size_t k = 0; k < static_cast<size_t>(num_tris); ++k)
    if (tri_indices[k] != other.tri_indices[k]) return false;

  for (size_t k = 0; k < static_cast<size_t>(num_vertices); ++k)
    if (vertices[k] != other.vertices[k]) return false;

  if (prev_vertices != nullptr && other.prev_vertices != nullptr) {
    for (size_t k = 0; k < static_cast<size_t>(num_vertices); ++k) {
      if (prev_vertices[k] != other.prev_vertices[k]) return false;
    }
  }

  return true;
}

void BVHModelBase::buildConvexRepresentation(bool share_memory) {
  if (!convex) {
    Vec3f* points = vertices;
    Triangle* polygons = tri_indices;
    if (!share_memory) {
      points = new Vec3f[num_vertices];
      std::copy(vertices, vertices + num_vertices, points);

      polygons = new Triangle[num_tris];
      std::copy(tri_indices, tri_indices + num_tris, polygons);
    }
    convex.reset(new Convex<Triangle>(!share_memory, points, num_vertices,
                                      polygons, num_tris));
  }
}

bool BVHModelBase::buildConvexHull(bool keepTriangle,
                                   const char* qhullCommand) {
  convex.reset(ConvexBase::convexHull(vertices, num_vertices, keepTriangle,
                                      qhullCommand));
  return num_vertices == convex->num_points;
}

template <typename BV>
BVHModel<BV>::BVHModel(const BVHModel<BV>& other)
    : BVHModelBase(other),
      bv_splitter(other.bv_splitter),
      bv_fitter(other.bv_fitter) {
  if (other.primitive_indices) {
    unsigned int num_primitives = 0;
    switch (other.getModelType()) {
      case BVH_MODEL_TRIANGLES:
        num_primitives = num_tris;
        break;
      case BVH_MODEL_POINTCLOUD:
        num_primitives = num_vertices;
        break;
      default:;
    }

    primitive_indices = new unsigned int[num_primitives];
    std::copy(other.primitive_indices, other.primitive_indices + num_primitives,
              primitive_indices);
  } else
    primitive_indices = nullptr;

  num_bvs = num_bvs_allocated = other.num_bvs;
  if (other.bvs) {
    bvs = new BVNode<BV>[num_bvs];
    std::copy(other.bvs, other.bvs + num_bvs, bvs);
  } else
    bvs = nullptr;
}

int BVHModelBase::beginModel(unsigned int num_tris_,
                             unsigned int num_vertices_) {
  if (build_state != BVH_BUILD_STATE_EMPTY) {
    delete[] vertices;
    vertices = nullptr;
    delete[] tri_indices;
    tri_indices = nullptr;
    delete[] prev_vertices;
    prev_vertices = nullptr;

    num_vertices_allocated = num_vertices = num_tris_allocated = num_tris = 0;
    deleteBVs();
  }

  if (num_tris_ <= 0) num_tris_ = 8;
  if (num_vertices_ <= 0) num_vertices_ = 8;

  num_vertices_allocated = num_vertices_;
  num_tris_allocated = num_tris_;

  tri_indices = new Triangle[num_tris_allocated];

  if (!tri_indices) {
    std::cerr << "BVH Error! Out of memory for tri_indices array on "
                 "BeginModel() call!"
              << std::endl;
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }

  vertices = new Vec3f[num_vertices_allocated];
  if (!vertices) {
    std::cerr
        << "BVH Error! Out of memory for vertices array on BeginModel() call!"
        << std::endl;
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }

  if (build_state != BVH_BUILD_STATE_EMPTY) {
    std::cerr
        << "BVH Warning! Call beginModel() on a BVHModel that is not empty. "
           "This model was cleared and previous triangles/vertices were lost."
        << std::endl;
    build_state = BVH_BUILD_STATE_EMPTY;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  build_state = BVH_BUILD_STATE_BEGUN;

  return BVH_OK;
}

int BVHModelBase::addVertex(const Vec3f& p) {
  if (build_state != BVH_BUILD_STATE_BEGUN) {
    std::cerr << "BVH Warning! Call addVertex() in a wrong order. addVertex() "
                 "was ignored. Must do a beginModel() to clear the model for "
                 "addition of new vertices."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if (num_vertices >= num_vertices_allocated) {
    Vec3f* temp = new Vec3f[num_vertices_allocated * 2];
    if (!temp) {
      std::cerr
          << "BVH Error! Out of memory for vertices array on addVertex() call!"
          << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(vertices, vertices + num_vertices, temp);
    delete[] vertices;
    vertices = temp;
    num_vertices_allocated *= 2;
  }

  vertices[num_vertices] = p;
  num_vertices += 1;

  return BVH_OK;
}

int BVHModelBase::addTriangles(const Matrixx3i& triangles) {
  if (build_state == BVH_BUILD_STATE_PROCESSED) {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. "
                 "addSubModel() was ignored. Must do a beginModel() to clear "
                 "the model for addition of new vertices."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  const unsigned int num_tris_to_add = (unsigned int)triangles.rows();

  if (num_tris + num_tris_to_add > num_tris_allocated) {
    Triangle* temp = new Triangle[num_tris_allocated * 2 + num_tris_to_add];
    if (!temp) {
      std::cerr << "BVH Error! Out of memory for tri_indices array on "
                   "addSubModel() call!"
                << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(tri_indices, tri_indices + num_tris, temp);
    delete[] tri_indices;
    tri_indices = temp;
    num_tris_allocated = num_tris_allocated * 2 + num_tris_to_add;
  }

  for (Eigen::DenseIndex i = 0; i < triangles.rows(); ++i) {
    const Matrixx3i::ConstRowXpr triangle = triangles.row(i);
    tri_indices[num_tris++].set(static_cast<Triangle::index_type>(triangle[0]),
                                static_cast<Triangle::index_type>(triangle[1]),
                                static_cast<Triangle::index_type>(triangle[2]));
  }

  return BVH_OK;
}

int BVHModelBase::addVertices(const Matrixx3f& points) {
  if (build_state != BVH_BUILD_STATE_BEGUN) {
    std::cerr << "BVH Warning! Call addVertex() in a wrong order. "
                 "addVertices() was ignored. Must do a beginModel() to clear "
                 "the model for addition of new vertices."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if (num_vertices + points.rows() > num_vertices_allocated) {
    num_vertices_allocated = num_vertices + (unsigned int)points.rows();
    Vec3f* temp = new Vec3f[num_vertices_allocated];
    if (!temp) {
      std::cerr
          << "BVH Error! Out of memory for vertices array on addVertex() call!"
          << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(vertices, vertices + num_vertices, temp);
    delete[] vertices;
    vertices = temp;
  }

  for (Eigen::DenseIndex id = 0; id < points.rows(); ++id)
    vertices[num_vertices++] = points.row(id).transpose();

  return BVH_OK;
}

int BVHModelBase::addTriangle(const Vec3f& p1, const Vec3f& p2,
                              const Vec3f& p3) {
  if (build_state == BVH_BUILD_STATE_PROCESSED) {
    std::cerr << "BVH Warning! Call addTriangle() in a wrong order. "
                 "addTriangle() was ignored. Must do a beginModel() to clear "
                 "the model for addition of new triangles."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if (num_vertices + 2 >= num_vertices_allocated) {
    Vec3f* temp = new Vec3f[num_vertices_allocated * 2 + 2];
    if (!temp) {
      std::cerr << "BVH Error! Out of memory for vertices array on "
                   "addTriangle() call!"
                << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(vertices, vertices + num_vertices, temp);
    delete[] vertices;
    vertices = temp;
    num_vertices_allocated = num_vertices_allocated * 2 + 2;
  }

  const unsigned int offset = num_vertices;

  vertices[num_vertices] = p1;
  num_vertices++;
  vertices[num_vertices] = p2;
  num_vertices++;
  vertices[num_vertices] = p3;
  num_vertices++;

  if (num_tris >= num_tris_allocated) {
    Triangle* temp = new Triangle[num_tris_allocated * 2];
    if (!temp) {
      std::cerr << "BVH Error! Out of memory for tri_indices array on "
                   "addTriangle() call!"
                << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(tri_indices, tri_indices + num_tris, temp);
    delete[] tri_indices;
    tri_indices = temp;
    num_tris_allocated *= 2;
  }

  tri_indices[num_tris].set((Triangle::index_type)offset,
                            (Triangle::index_type)(offset + 1),
                            (Triangle::index_type)(offset + 2));
  num_tris++;

  return BVH_OK;
}

int BVHModelBase::addSubModel(const std::vector<Vec3f>& ps) {
  if (build_state == BVH_BUILD_STATE_PROCESSED) {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. "
                 "addSubModel() was ignored. Must do a beginModel() to clear "
                 "the model for addition of new vertices."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  const unsigned int num_vertices_to_add = (unsigned int)ps.size();

  if (num_vertices + num_vertices_to_add - 1 >= num_vertices_allocated) {
    Vec3f* temp =
        new Vec3f[num_vertices_allocated * 2 + num_vertices_to_add - 1];
    if (!temp) {
      std::cerr << "BVH Error! Out of memory for vertices array on "
                   "addSubModel() call!"
                << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(vertices, vertices + num_vertices, temp);
    delete[] vertices;
    vertices = temp;
    num_vertices_allocated =
        num_vertices_allocated * 2 + num_vertices_to_add - 1;
  }

  for (size_t i = 0; i < (size_t)num_vertices_to_add; ++i) {
    vertices[num_vertices] = ps[i];
    num_vertices++;
  }

  return BVH_OK;
}

int BVHModelBase::addSubModel(const std::vector<Vec3f>& ps,
                              const std::vector<Triangle>& ts) {
  if (build_state == BVH_BUILD_STATE_PROCESSED) {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. "
                 "addSubModel() was ignored. Must do a beginModel() to clear "
                 "the model for addition of new vertices."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  const unsigned int num_vertices_to_add = (unsigned int)ps.size();

  if (num_vertices + num_vertices_to_add - 1 >= num_vertices_allocated) {
    Vec3f* temp =
        new Vec3f[num_vertices_allocated * 2 + num_vertices_to_add - 1];
    if (!temp) {
      std::cerr << "BVH Error! Out of memory for vertices array on "
                   "addSubModel() call!"
                << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(vertices, vertices + num_vertices, temp);
    delete[] vertices;
    vertices = temp;
    num_vertices_allocated =
        num_vertices_allocated * 2 + num_vertices_to_add - 1;
  }

  const unsigned int offset = num_vertices;

  for (size_t i = 0; i < (size_t)num_vertices_to_add; ++i) {
    vertices[num_vertices] = ps[i];
    num_vertices++;
  }

  const unsigned int num_tris_to_add = (unsigned int)ts.size();

  if (num_tris + num_tris_to_add - 1 >= num_tris_allocated) {
    Triangle* temp = new Triangle[num_tris_allocated * 2 + num_tris_to_add - 1];
    if (!temp) {
      std::cerr << "BVH Error! Out of memory for tri_indices array on "
                   "addSubModel() call!"
                << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }

    std::copy(tri_indices, tri_indices + num_tris, temp);
    delete[] tri_indices;
    tri_indices = temp;
    num_tris_allocated = num_tris_allocated * 2 + num_tris_to_add - 1;
  }

  for (size_t i = 0; i < (size_t)num_tris_to_add; ++i) {
    const Triangle& t = ts[i];
    tri_indices[num_tris].set(t[0] + (size_t)offset, t[1] + (size_t)offset,
                              t[2] + (size_t)offset);
    num_tris++;
  }

  return BVH_OK;
}

int BVHModelBase::endModel() {
  if (build_state != BVH_BUILD_STATE_BEGUN) {
    std::cerr << "BVH Warning! Call endModel() in wrong order. endModel() was "
                 "ignored."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if (num_tris == 0 && num_vertices == 0) {
    std::cerr << "BVH Error! endModel() called on model with no triangles and "
                 "vertices."
              << std::endl;
    return BVH_ERR_BUILD_EMPTY_MODEL;
  }

  if (num_tris_allocated > num_tris) {
    if (num_tris > 0) {
      Triangle* new_tris = new Triangle[num_tris];
      if (!new_tris) {
        std::cerr << "BVH Error! Out of memory for tri_indices array in "
                     "endModel() call!"
                  << std::endl;
        return BVH_ERR_MODEL_OUT_OF_MEMORY;
      }
      std::copy(tri_indices, tri_indices + num_tris, new_tris);
      delete[] tri_indices;
      tri_indices = new_tris;
      num_tris_allocated = num_tris;
    } else {
      delete[] tri_indices;
      tri_indices = NULL;
      num_tris = num_tris_allocated = 0;
    }
  }

  if (num_vertices_allocated > num_vertices) {
    Vec3f* new_vertices = new Vec3f[num_vertices];
    if (!new_vertices) {
      std::cerr
          << "BVH Error! Out of memory for vertices array in endModel() call!"
          << std::endl;
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }
    std::copy(vertices, vertices + num_vertices, new_vertices);
    delete[] vertices;
    vertices = new_vertices;
    num_vertices_allocated = num_vertices;
  }

  // construct BVH tree
  if (!allocateBVs()) return BVH_ERR_MODEL_OUT_OF_MEMORY;

  buildTree();

  // finish constructing
  build_state = BVH_BUILD_STATE_PROCESSED;

  return BVH_OK;
}

int BVHModelBase::beginReplaceModel() {
  if (build_state != BVH_BUILD_STATE_PROCESSED) {
    std::cerr << "BVH Error! Call beginReplaceModel() on a BVHModel that has "
                 "no previous frame."
              << std::endl;
    return BVH_ERR_BUILD_EMPTY_PREVIOUS_FRAME;
  }

  if (prev_vertices) delete[] prev_vertices;
  prev_vertices = NULL;

  num_vertex_updated = 0;

  build_state = BVH_BUILD_STATE_REPLACE_BEGUN;

  return BVH_OK;
}

int BVHModelBase::replaceVertex(const Vec3f& p) {
  if (build_state != BVH_BUILD_STATE_REPLACE_BEGUN) {
    std::cerr << "BVH Warning! Call replaceVertex() in a wrong order. "
                 "replaceVertex() was ignored. Must do a beginReplaceModel() "
                 "for initialization."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p;
  num_vertex_updated++;

  return BVH_OK;
}

int BVHModelBase::replaceTriangle(const Vec3f& p1, const Vec3f& p2,
                                  const Vec3f& p3) {
  if (build_state != BVH_BUILD_STATE_REPLACE_BEGUN) {
    std::cerr << "BVH Warning! Call replaceTriangle() in a wrong order. "
                 "replaceTriangle() was ignored. Must do a beginReplaceModel() "
                 "for initialization."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p1;
  num_vertex_updated++;
  vertices[num_vertex_updated] = p2;
  num_vertex_updated++;
  vertices[num_vertex_updated] = p3;
  num_vertex_updated++;
  return BVH_OK;
}

int BVHModelBase::replaceSubModel(const std::vector<Vec3f>& ps) {
  if (build_state != BVH_BUILD_STATE_REPLACE_BEGUN) {
    std::cerr << "BVH Warning! Call replaceSubModel() in a wrong order. "
                 "replaceSubModel() was ignored. Must do a beginReplaceModel() "
                 "for initialization."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  for (unsigned int i = 0; i < ps.size(); ++i) {
    vertices[num_vertex_updated] = ps[i];
    num_vertex_updated++;
  }
  return BVH_OK;
}

int BVHModelBase::endReplaceModel(bool refit, bool bottomup) {
  if (build_state != BVH_BUILD_STATE_REPLACE_BEGUN) {
    std::cerr << "BVH Warning! Call endReplaceModel() in a wrong order. "
                 "endReplaceModel() was ignored. "
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if (num_vertex_updated != num_vertices) {
    std::cerr << "BVH Error! The replaced model should have the same number of "
                 "vertices as the old model."
              << std::endl;
    return BVH_ERR_INCORRECT_DATA;
  }

  if (refit)  // refit, do not change BVH structure
  {
    refitTree(bottomup);
  } else  // reconstruct bvh tree based on current frame data
  {
    buildTree();
  }

  build_state = BVH_BUILD_STATE_PROCESSED;

  return BVH_OK;
}

int BVHModelBase::beginUpdateModel() {
  if (build_state != BVH_BUILD_STATE_PROCESSED &&
      build_state != BVH_BUILD_STATE_UPDATED) {
    std::cerr << "BVH Error! Call beginUpdatemodel() on a BVHModel that has no "
                 "previous frame."
              << std::endl;
    return BVH_ERR_BUILD_EMPTY_PREVIOUS_FRAME;
  }

  if (prev_vertices) {
    Vec3f* temp = prev_vertices;
    prev_vertices = vertices;
    vertices = temp;
  } else {
    prev_vertices = vertices;
    vertices = new Vec3f[num_vertices];
  }

  num_vertex_updated = 0;

  build_state = BVH_BUILD_STATE_UPDATE_BEGUN;

  return BVH_OK;
}

int BVHModelBase::updateVertex(const Vec3f& p) {
  if (build_state != BVH_BUILD_STATE_UPDATE_BEGUN) {
    std::cerr
        << "BVH Warning! Call updateVertex() in a wrong order. updateVertex() "
           "was ignored. Must do a beginUpdateModel() for initialization."
        << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p;
  num_vertex_updated++;

  return BVH_OK;
}

int BVHModelBase::updateTriangle(const Vec3f& p1, const Vec3f& p2,
                                 const Vec3f& p3) {
  if (build_state != BVH_BUILD_STATE_UPDATE_BEGUN) {
    std::cerr << "BVH Warning! Call updateTriangle() in a wrong order. "
                 "updateTriangle() was ignored. Must do a beginUpdateModel() "
                 "for initialization."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p1;
  num_vertex_updated++;
  vertices[num_vertex_updated] = p2;
  num_vertex_updated++;
  vertices[num_vertex_updated] = p3;
  num_vertex_updated++;
  return BVH_OK;
}

int BVHModelBase::updateSubModel(const std::vector<Vec3f>& ps) {
  if (build_state != BVH_BUILD_STATE_UPDATE_BEGUN) {
    std::cerr << "BVH Warning! Call updateSubModel() in a wrong order. "
                 "updateSubModel() was ignored. Must do a beginUpdateModel() "
                 "for initialization."
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  for (unsigned int i = 0; i < ps.size(); ++i) {
    vertices[num_vertex_updated] = ps[i];
    num_vertex_updated++;
  }
  return BVH_OK;
}

int BVHModelBase::endUpdateModel(bool refit, bool bottomup) {
  if (build_state != BVH_BUILD_STATE_UPDATE_BEGUN) {
    std::cerr << "BVH Warning! Call endUpdateModel() in a wrong order. "
                 "endUpdateModel() was ignored. "
              << std::endl;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if (num_vertex_updated != num_vertices) {
    std::cerr << "BVH Error! The updated model should have the same number of "
                 "vertices as the old model."
              << std::endl;
    return BVH_ERR_INCORRECT_DATA;
  }

  if (refit)  // refit, do not change BVH structure
  {
    refitTree(bottomup);
  } else  // reconstruct bvh tree based on current frame data
  {
    buildTree();

    // then refit

    refitTree(bottomup);
  }

  build_state = BVH_BUILD_STATE_UPDATED;

  return BVH_OK;
}

void BVHModelBase::computeLocalAABB() {
  AABB aabb_;
  for (unsigned int i = 0; i < num_vertices; ++i) {
    aabb_ += vertices[i];
  }

  aabb_center = aabb_.center();

  aabb_radius = 0;
  for (unsigned int i = 0; i < num_vertices; ++i) {
    FCL_REAL r = (aabb_center - vertices[i]).squaredNorm();
    if (r > aabb_radius) aabb_radius = r;
  }

  aabb_radius = sqrt(aabb_radius);

  aabb_local = aabb_;
}

/// @brief Constructing an empty BVH
template <typename BV>
BVHModel<BV>::BVHModel()
    : BVHModelBase(),
      bv_splitter(new BVSplitter<BV>(SPLIT_METHOD_MEAN)),
      bv_fitter(new BVFitter<BV>()),
      num_bvs_allocated(0),
      primitive_indices(NULL),
      bvs(NULL),
      num_bvs(0) {}

template <typename BV>
void BVHModel<BV>::deleteBVs() {
  delete[] bvs;
  bvs = NULL;
  delete[] primitive_indices;
  primitive_indices = NULL;
  num_bvs_allocated = num_bvs = 0;
}

template <typename BV>
bool BVHModel<BV>::allocateBVs() {
  // construct BVH tree
  unsigned int num_bvs_to_be_allocated = 0;
  if (num_tris == 0)
    num_bvs_to_be_allocated = 2 * num_vertices - 1;
  else
    num_bvs_to_be_allocated = 2 * num_tris - 1;

  bvs = new BVNode<BV>[num_bvs_to_be_allocated];
  primitive_indices = new unsigned int[num_bvs_to_be_allocated];
  if (!bvs || !primitive_indices) {
    std::cerr << "BVH Error! Out of memory for BV array in endModel()!"
              << std::endl;
    return false;
  }
  num_bvs_allocated = num_bvs_to_be_allocated;
  num_bvs = 0;
  return true;
}

template <typename BV>
int BVHModel<BV>::memUsage(const bool msg) const {
  unsigned int mem_bv_list = (unsigned int)sizeof(BV) * num_bvs;
  unsigned int mem_tri_list = (unsigned int)sizeof(Triangle) * num_tris;
  unsigned int mem_vertex_list = (unsigned int)sizeof(Vec3f) * num_vertices;

  unsigned int total_mem = mem_bv_list + mem_tri_list + mem_vertex_list +
                           (unsigned int)sizeof(BVHModel<BV>);
  if (msg) {
    std::cerr << "Total for model " << total_mem << " bytes." << std::endl;
    std::cerr << "BVs: " << num_bvs << " allocated." << std::endl;
    std::cerr << "Tris: " << num_tris << " allocated." << std::endl;
    std::cerr << "Vertices: " << num_vertices << " allocated." << std::endl;
  }

  return static_cast<int>(total_mem);
}

template <typename BV>
int BVHModel<BV>::buildTree() {
  // set BVFitter
  bv_fitter->set(vertices, tri_indices, getModelType());
  // set SplitRule
  bv_splitter->set(vertices, tri_indices, getModelType());

  num_bvs = 1;

  unsigned int num_primitives = 0;
  switch (getModelType()) {
    case BVH_MODEL_TRIANGLES:
      num_primitives = (unsigned int)num_tris;
      break;
    case BVH_MODEL_POINTCLOUD:
      num_primitives = (unsigned int)num_vertices;
      break;
    default:
      std::cerr << "BVH Error: Model type not supported!" << std::endl;
      return BVH_ERR_UNSUPPORTED_FUNCTION;
  }

  for (unsigned int i = 0; i < num_primitives; ++i) primitive_indices[i] = i;
  recursiveBuildTree(0, 0, num_primitives);

  bv_fitter->clear();
  bv_splitter->clear();

  return BVH_OK;
}

template <typename BV>
int BVHModel<BV>::recursiveBuildTree(int bv_id, unsigned int first_primitive,
                                     unsigned int num_primitives) {
  BVHModelType type = getModelType();
  BVNode<BV>* bvnode = bvs + bv_id;
  unsigned int* cur_primitive_indices = primitive_indices + first_primitive;

  // constructing BV
  BV bv = bv_fitter->fit(cur_primitive_indices, num_primitives);
  bv_splitter->computeRule(bv, cur_primitive_indices, num_primitives);

  bvnode->bv = bv;
  bvnode->first_primitive = first_primitive;
  bvnode->num_primitives = num_primitives;

  if (num_primitives == 1) {
    bvnode->first_child = -((int)(*cur_primitive_indices) + 1);
  } else {
    bvnode->first_child = (int)num_bvs;
    num_bvs += 2;

    unsigned int c1 = 0;
    for (unsigned int i = 0; i < num_primitives; ++i) {
      Vec3f p;
      if (type == BVH_MODEL_POINTCLOUD)
        p = vertices[cur_primitive_indices[i]];
      else if (type == BVH_MODEL_TRIANGLES) {
        const Triangle& t = tri_indices[cur_primitive_indices[i]];
        const Vec3f& p1 = vertices[t[0]];
        const Vec3f& p2 = vertices[t[1]];
        const Vec3f& p3 = vertices[t[2]];

        p = (p1 + p2 + p3) / 3.;
      } else {
        std::cerr << "BVH Error: Model type not supported!" << std::endl;
        return BVH_ERR_UNSUPPORTED_FUNCTION;
      }

      // loop invariant: up to (but not including) index c1 in group 1,
      // then up to (but not including) index i in group 2
      //
      //  [1] [1] [1] [1] [2] [2] [2] [x] [x] ... [x]
      //                   c1          i
      //
      if (bv_splitter->apply(p))  // in the right side
      {
        // do nothing
      } else {
        unsigned int temp = cur_primitive_indices[i];
        cur_primitive_indices[i] = cur_primitive_indices[c1];
        cur_primitive_indices[c1] = temp;
        c1++;
      }
    }

    if ((c1 == 0) || (c1 == num_primitives)) c1 = num_primitives / 2;

    const unsigned int num_first_half = c1;

    recursiveBuildTree(bvnode->leftChild(), first_primitive, num_first_half);
    recursiveBuildTree(bvnode->rightChild(), first_primitive + num_first_half,
                       num_primitives - num_first_half);
  }

  return BVH_OK;
}

template <typename BV>
int BVHModel<BV>::refitTree(bool bottomup) {
  if (bottomup)
    return refitTree_bottomup();
  else
    return refitTree_topdown();
}

template <typename BV>
int BVHModel<BV>::refitTree_bottomup() {
  // TODO the recomputation of the BV is done manually, without using
  // bv_fitter. The manual BV recomputation seems bugged. Using bv_fitter
  // seems to correct the bug.
  // bv_fitter->set(vertices, tri_indices, getModelType());

  int res = recursiveRefitTree_bottomup(0);

  // bv_fitter->clear();
  return res;
}

template <typename BV>
int BVHModel<BV>::recursiveRefitTree_bottomup(int bv_id) {
  BVNode<BV>* bvnode = bvs + bv_id;
  if (bvnode->isLeaf()) {
    BVHModelType type = getModelType();
    int primitive_id = -(bvnode->first_child + 1);
    if (type == BVH_MODEL_POINTCLOUD) {
      BV bv;

      if (prev_vertices) {
        Vec3f v[2];
        v[0] = prev_vertices[primitive_id];
        v[1] = vertices[primitive_id];
        fit(v, 2, bv);
      } else
        fit(vertices + primitive_id, 1, bv);

      bvnode->bv = bv;
    } else if (type == BVH_MODEL_TRIANGLES) {
      BV bv;
      const Triangle& triangle = tri_indices[primitive_id];

      if (prev_vertices) {
        Vec3f v[6];
        for (Triangle::index_type i = 0; i < 3; ++i) {
          v[i] = prev_vertices[triangle[i]];
          v[i + 3] = vertices[triangle[i]];
        }

        fit(v, 6, bv);
      } else {
        // TODO use bv_fitter to build BV. See comment in refitTree_bottomup
        // unsigned int* cur_primitive_indices = primitive_indices +
        // bvnode->first_primitive; bv = bv_fitter->fit(cur_primitive_indices,
        // bvnode->num_primitives);
        Vec3f v[3];
        for (int i = 0; i < 3; ++i) {
          v[i] = vertices[triangle[(Triangle::index_type)i]];
        }

        fit(v, 3, bv);
      }

      bvnode->bv = bv;
    } else {
      std::cerr << "BVH Error: Model type not supported!" << std::endl;
      return BVH_ERR_UNSUPPORTED_FUNCTION;
    }
  } else {
    recursiveRefitTree_bottomup(bvnode->leftChild());
    recursiveRefitTree_bottomup(bvnode->rightChild());
    bvnode->bv = bvs[bvnode->leftChild()].bv + bvs[bvnode->rightChild()].bv;
    // TODO use bv_fitter to build BV. See comment in refitTree_bottomup
    // unsigned int* cur_primitive_indices = primitive_indices +
    // bvnode->first_primitive; bvnode->bv =
    // bv_fitter->fit(cur_primitive_indices, bvnode->num_primitives);
  }

  return BVH_OK;
}

template <typename BV>
int BVHModel<BV>::refitTree_topdown() {
  bv_fitter->set(vertices, prev_vertices, tri_indices, getModelType());
  for (unsigned int i = 0; i < num_bvs; ++i) {
    BV bv = bv_fitter->fit(primitive_indices + bvs[i].first_primitive,
                           bvs[i].num_primitives);
    bvs[i].bv = bv;
  }

  bv_fitter->clear();

  return BVH_OK;
}

template <>
void BVHModel<OBB>::makeParentRelativeRecurse(int bv_id, Matrix3f& parent_axes,
                                              const Vec3f& parent_c) {
  OBB& obb = bvs[bv_id].bv;
  if (!bvs[bv_id].isLeaf()) {
    makeParentRelativeRecurse(bvs[bv_id].first_child, obb.axes, obb.To);

    makeParentRelativeRecurse(bvs[bv_id].first_child + 1, obb.axes, obb.To);
  }

  // make self parent relative
  // obb.axes = parent_axes.transpose() * obb.axes;
  obb.axes.applyOnTheLeft(parent_axes.transpose());

  Vec3f t(obb.To - parent_c);
  obb.To.noalias() = parent_axes.transpose() * t;
}

template <>
void BVHModel<RSS>::makeParentRelativeRecurse(int bv_id, Matrix3f& parent_axes,
                                              const Vec3f& parent_c) {
  RSS& rss = bvs[bv_id].bv;
  if (!bvs[bv_id].isLeaf()) {
    makeParentRelativeRecurse(bvs[bv_id].first_child, rss.axes, rss.Tr);

    makeParentRelativeRecurse(bvs[bv_id].first_child + 1, rss.axes, rss.Tr);
  }

  // make self parent relative
  // rss.axes = parent_axes.transpose() * rss.axes;
  rss.axes.applyOnTheLeft(parent_axes.transpose());

  Vec3f t(rss.Tr - parent_c);
  rss.Tr.noalias() = parent_axes.transpose() * t;
}

template <>
void BVHModel<OBBRSS>::makeParentRelativeRecurse(int bv_id,
                                                 Matrix3f& parent_axes,
                                                 const Vec3f& parent_c) {
  OBB& obb = bvs[bv_id].bv.obb;
  RSS& rss = bvs[bv_id].bv.rss;
  if (!bvs[bv_id].isLeaf()) {
    makeParentRelativeRecurse(bvs[bv_id].first_child, obb.axes, obb.To);

    makeParentRelativeRecurse(bvs[bv_id].first_child + 1, obb.axes, obb.To);
  }

  // make self parent relative
  rss.axes.noalias() = parent_axes.transpose() * obb.axes;
  obb.axes = rss.axes;

  Vec3f t(obb.To - parent_c);
  obb.To.noalias() = parent_axes.transpose() * t;
  rss.Tr = obb.To;
}

template <>
NODE_TYPE BVHModel<AABB>::getNodeType() const {
  return BV_AABB;
}

template <>
NODE_TYPE BVHModel<OBB>::getNodeType() const {
  return BV_OBB;
}

template <>
NODE_TYPE BVHModel<RSS>::getNodeType() const {
  return BV_RSS;
}

template <>
NODE_TYPE BVHModel<kIOS>::getNodeType() const {
  return BV_kIOS;
}

template <>
NODE_TYPE BVHModel<OBBRSS>::getNodeType() const {
  return BV_OBBRSS;
}

template <>
NODE_TYPE BVHModel<KDOP<16> >::getNodeType() const {
  return BV_KDOP16;
}

template <>
NODE_TYPE BVHModel<KDOP<18> >::getNodeType() const {
  return BV_KDOP18;
}

template <>
NODE_TYPE BVHModel<KDOP<24> >::getNodeType() const {
  return BV_KDOP24;
}

template class BVHModel<KDOP<16> >;
template class BVHModel<KDOP<18> >;
template class BVHModel<KDOP<24> >;
template class BVHModel<OBB>;
template class BVHModel<AABB>;
template class BVHModel<RSS>;
template class BVHModel<kIOS>;
template class BVHModel<OBBRSS>;

}  // namespace fcl

}  // namespace hpp
