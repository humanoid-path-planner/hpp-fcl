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

#ifndef HPP_FCL_TRAVERSAL_NODE_OCTREE_H
#define HPP_FCL_TRAVERSAL_NODE_OCTREE_H

/// @cond INTERNAL

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/internal/traversal_node_base.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/octree.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes_utility.h>
#include <hpp/fcl/internal/shape_shape_func.h>

namespace hpp {
namespace fcl {

/// @brief Algorithms for collision related with octree
class HPP_FCL_DLLAPI OcTreeSolver {
 private:
  const GJKSolver* solver;

  mutable const CollisionRequest* crequest;
  mutable const DistanceRequest* drequest;

  mutable CollisionResult* cresult;
  mutable DistanceResult* dresult;

 public:
  OcTreeSolver(const GJKSolver* solver_)
      : solver(solver_),
        crequest(NULL),
        drequest(NULL),
        cresult(NULL),
        dresult(NULL) {}

  /// @brief collision between two octrees
  void OcTreeIntersect(const OcTree* tree1, const OcTree* tree2,
                       const Transform3f& tf1, const Transform3f& tf2,
                       const CollisionRequest& request_,
                       CollisionResult& result_) const {
    crequest = &request_;
    cresult = &result_;

    OcTreeIntersectRecurse(tree1, tree1->getRoot(), tree1->getRootBV(), tree2,
                           tree2->getRoot(), tree2->getRootBV(), tf1, tf2);
  }

  /// @brief distance between two octrees
  void OcTreeDistance(const OcTree* tree1, const OcTree* tree2,
                      const Transform3f& tf1, const Transform3f& tf2,
                      const DistanceRequest& request_,
                      DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    OcTreeDistanceRecurse(tree1, tree1->getRoot(), tree1->getRootBV(), tree2,
                          tree2->getRoot(), tree2->getRootBV(), tf1, tf2);
  }

  /// @brief collision between octree and mesh
  template <typename BV>
  void OcTreeMeshIntersect(const OcTree* tree1, const BVHModel<BV>* tree2,
                           const Transform3f& tf1, const Transform3f& tf2,
                           const CollisionRequest& request_,
                           CollisionResult& result_) const {
    crequest = &request_;
    cresult = &result_;

    OcTreeMeshIntersectRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                               tree2, 0, tf1, tf2);
  }

  /// @brief distance between octree and mesh
  template <typename BV>
  void OcTreeMeshDistance(const OcTree* tree1, const BVHModel<BV>* tree2,
                          const Transform3f& tf1, const Transform3f& tf2,
                          const DistanceRequest& request_,
                          DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    OcTreeMeshDistanceRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                              tree2, 0, tf1, tf2);
  }

  /// @brief collision between mesh and octree
  template <typename BV>
  void MeshOcTreeIntersect(const BVHModel<BV>* tree1, const OcTree* tree2,
                           const Transform3f& tf1, const Transform3f& tf2,
                           const CollisionRequest& request_,
                           CollisionResult& result_) const

  {
    crequest = &request_;
    cresult = &result_;

    OcTreeMeshIntersectRecurse(tree2, tree2->getRoot(), tree2->getRootBV(),
                               tree1, 0, tf2, tf1);
  }

  /// @brief distance between mesh and octree
  template <typename BV>
  void MeshOcTreeDistance(const BVHModel<BV>* tree1, const OcTree* tree2,
                          const Transform3f& tf1, const Transform3f& tf2,
                          const DistanceRequest& request_,
                          DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    OcTreeMeshDistanceRecurse(tree1, 0, tree2, tree2->getRoot(),
                              tree2->getRootBV(), tf1, tf2);
  }

  /// @brief collision between octree and shape
  template <typename S>
  void OcTreeShapeIntersect(const OcTree* tree, const S& s,
                            const Transform3f& tf1, const Transform3f& tf2,
                            const CollisionRequest& request_,
                            CollisionResult& result_) const {
    crequest = &request_;
    cresult = &result_;

    AABB bv2;
    computeBV<AABB>(s, Transform3f(), bv2);
    OBB obb2;
    convertBV(bv2, tf2, obb2);
    OcTreeShapeIntersectRecurse(tree, tree->getRoot(), tree->getRootBV(), s,
                                obb2, tf1, tf2);
  }

  /// @brief collision between shape and octree
  template <typename S>
  void ShapeOcTreeIntersect(const S& s, const OcTree* tree,
                            const Transform3f& tf1, const Transform3f& tf2,
                            const CollisionRequest& request_,
                            CollisionResult& result_) const {
    crequest = &request_;
    cresult = &result_;

    AABB bv1;
    computeBV<AABB>(s, Transform3f(), bv1);
    OBB obb1;
    convertBV(bv1, tf1, obb1);
    OcTreeShapeIntersectRecurse(tree, tree->getRoot(), tree->getRootBV(), s,
                                obb1, tf2, tf1);
  }

  /// @brief distance between octree and shape
  template <typename S>
  void OcTreeShapeDistance(const OcTree* tree, const S& s,
                           const Transform3f& tf1, const Transform3f& tf2,
                           const DistanceRequest& request_,
                           DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    AABB aabb2;
    computeBV<AABB>(s, tf2, aabb2);
    OcTreeShapeDistanceRecurse(tree, tree->getRoot(), tree->getRootBV(), s,
                               aabb2, tf1, tf2);
  }

  /// @brief distance between shape and octree
  template <typename S>
  void ShapeOcTreeDistance(const S& s, const OcTree* tree,
                           const Transform3f& tf1, const Transform3f& tf2,
                           const DistanceRequest& request_,
                           DistanceResult& result_) const {
    drequest = &request_;
    dresult = &result_;

    AABB aabb1;
    computeBV<AABB>(s, tf1, aabb1);
    OcTreeShapeDistanceRecurse(tree, tree->getRoot(), tree->getRootBV(), s,
                               aabb1, tf2, tf1);
  }

 private:
  template <typename S>
  bool OcTreeShapeDistanceRecurse(const OcTree* tree1,
                                  const OcTree::OcTreeNode* root1,
                                  const AABB& bv1, const S& s,
                                  const AABB& aabb2, const Transform3f& tf1,
                                  const Transform3f& tf2) const {
    if (!tree1->nodeHasChildren(root1)) {
      if (tree1->isNodeOccupied(root1)) {
        Box box;
        Transform3f box_tf;
        constructBox(bv1, tf1, box, box_tf);

        FCL_REAL dist;
        Vec3f closest_p1, closest_p2, normal;
        solver->shapeDistance(box, box_tf, s, tf2, dist, closest_p1, closest_p2,
                              normal);

        dresult->update(dist, tree1, &s, (int)(root1 - tree1->getRoot()),
                        DistanceResult::NONE, closest_p1, closest_p2, normal);

        return drequest->isSatisfied(*dresult);
      } else
        return false;
    }

    if (!tree1->isNodeOccupied(root1)) return false;

    for (unsigned int i = 0; i < 8; ++i) {
      if (tree1->nodeChildExists(root1, i)) {
        const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
        AABB child_bv;
        computeChildBV(bv1, i, child_bv);

        AABB aabb1;
        convertBV(child_bv, tf1, aabb1);
        FCL_REAL d = aabb1.distance(aabb2);
        if (d < dresult->min_distance) {
          if (OcTreeShapeDistanceRecurse(tree1, child, child_bv, s, aabb2, tf1,
                                         tf2))
            return true;
        }
      }
    }

    return false;
  }

  template <typename S>
  bool OcTreeShapeIntersectRecurse(const OcTree* tree1,
                                   const OcTree::OcTreeNode* root1,
                                   const AABB& bv1, const S& s, const OBB& obb2,
                                   const Transform3f& tf1,
                                   const Transform3f& tf2) const {
    // Empty OcTree is considered free.
    if (!root1) return false;

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least of one the nodes is free; OR
    ///           2) (two uncertain nodes or one node occupied and one node
    ///           uncertain) AND cost not required
    if (tree1->isNodeFree(root1))
      return false;
    else if ((tree1->isNodeUncertain(root1) || s.isUncertain()))
      return false;
    else {
      OBB obb1;
      convertBV(bv1, tf1, obb1);
      FCL_REAL sqrDistLowerBound;
      if (!obb1.overlap(obb2, *crequest, sqrDistLowerBound)) {
        internal::updateDistanceLowerBoundFromBV(*crequest, *cresult,
                                                 sqrDistLowerBound);
        return false;
      }
    }

    if (!tree1->nodeHasChildren(root1)) {
      assert(tree1->isNodeOccupied(root1));  // it isn't free nor uncertain.

      Box box;
      Transform3f box_tf;
      constructBox(bv1, tf1, box, box_tf);

      bool contactNotAdded =
          (cresult->numContacts() >= crequest->num_max_contacts);
      std::size_t ncontact = ShapeShapeCollide<Box, S>(
          &box, box_tf, &s, tf2, solver, *crequest, *cresult);
      assert(ncontact == 0 || ncontact == 1);
      if (!contactNotAdded && ncontact == 1) {
        // Update contact information.
        const Contact& c = cresult->getContact(cresult->numContacts() - 1);
        cresult->setContact(
            cresult->numContacts() - 1,
            Contact(tree1, c.o2, static_cast<int>(root1 - tree1->getRoot()),
                    c.b2, c.pos, c.normal, c.penetration_depth));
      }

      return crequest->isSatisfied(*cresult);
    }

    for (unsigned int i = 0; i < 8; ++i) {
      if (tree1->nodeChildExists(root1, i)) {
        const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
        AABB child_bv;
        computeChildBV(bv1, i, child_bv);

        if (OcTreeShapeIntersectRecurse(tree1, child, child_bv, s, obb2, tf1,
                                        tf2))
          return true;
      }
    }

    return false;
  }

  template <typename BV>
  bool OcTreeMeshDistanceRecurse(const OcTree* tree1,
                                 const OcTree::OcTreeNode* root1,
                                 const AABB& bv1, const BVHModel<BV>* tree2,
                                 unsigned int root2, const Transform3f& tf1,
                                 const Transform3f& tf2) const {
    if (!tree1->nodeHasChildren(root1) && tree2->getBV(root2).isLeaf()) {
      if (tree1->isNodeOccupied(root1)) {
        Box box;
        Transform3f box_tf;
        constructBox(bv1, tf1, box, box_tf);

        int primitive_id = tree2->getBV(root2).primitiveId();
        const Triangle& tri_id = tree2->tri_indices[primitive_id];
        const Vec3f& p1 = tree2->vertices[tri_id[0]];
        const Vec3f& p2 = tree2->vertices[tri_id[1]];
        const Vec3f& p3 = tree2->vertices[tri_id[2]];

        FCL_REAL dist;
        Vec3f closest_p1, closest_p2, normal;
        solver->shapeTriangleInteraction(box, box_tf, p1, p2, p3, tf2, dist,
                                         closest_p1, closest_p2, normal);

        dresult->update(dist, tree1, tree2, (int)(root1 - tree1->getRoot()),
                        primitive_id, closest_p1, closest_p2, normal);

        return drequest->isSatisfied(*dresult);
      } else
        return false;
    }

    if (!tree1->isNodeOccupied(root1)) return false;

    if (tree2->getBV(root2).isLeaf() ||
        (tree1->nodeHasChildren(root1) &&
         (bv1.size() > tree2->getBV(root2).bv.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree1->nodeChildExists(root1, i)) {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          FCL_REAL d;
          AABB aabb1, aabb2;
          convertBV(child_bv, tf1, aabb1);
          convertBV(tree2->getBV(root2).bv, tf2, aabb2);
          d = aabb1.distance(aabb2);

          if (d < dresult->min_distance) {
            if (OcTreeMeshDistanceRecurse(tree1, child, child_bv, tree2, root2,
                                          tf1, tf2))
              return true;
          }
        }
      }
    } else {
      FCL_REAL d;
      AABB aabb1, aabb2;
      convertBV(bv1, tf1, aabb1);
      unsigned int child = (unsigned int)tree2->getBV(root2).leftChild();
      convertBV(tree2->getBV(child).bv, tf2, aabb2);
      d = aabb1.distance(aabb2);

      if (d < dresult->min_distance) {
        if (OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, child, tf1,
                                      tf2))
          return true;
      }

      child = (unsigned int)tree2->getBV(root2).rightChild();
      convertBV(tree2->getBV(child).bv, tf2, aabb2);
      d = aabb1.distance(aabb2);

      if (d < dresult->min_distance) {
        if (OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, child, tf1,
                                      tf2))
          return true;
      }
    }

    return false;
  }

  /// \return True if the request is satisfied.
  template <typename BV>
  bool OcTreeMeshIntersectRecurse(const OcTree* tree1,
                                  const OcTree::OcTreeNode* root1,
                                  const AABB& bv1, const BVHModel<BV>* tree2,
                                  unsigned int root2, const Transform3f& tf1,
                                  const Transform3f& tf2) const {
    // FIXME(jmirabel) I do not understand why the BVHModel was traversed. The
    // code in this if(!root1) did not output anything so the empty OcTree is
    // considered free. Should an empty OcTree be considered free ?

    // Empty OcTree is considered free.
    if (!root1) return false;
    BVNode<BV> const& bvn2 = tree2->getBV(root2);

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least one of the nodes is free; OR
    ///           2) (two uncertain nodes OR one node occupied and one node
    ///           uncertain) AND cost not required
    if (tree1->isNodeFree(root1))
      return false;
    else if ((tree1->isNodeUncertain(root1) || tree2->isUncertain()))
      return false;
    else {
      OBB obb1, obb2;
      convertBV(bv1, tf1, obb1);
      convertBV(bvn2.bv, tf2, obb2);
      FCL_REAL sqrDistLowerBound;
      if (!obb1.overlap(obb2, *crequest, sqrDistLowerBound)) {
        internal::updateDistanceLowerBoundFromBV(*crequest, *cresult,
                                                 sqrDistLowerBound);
        return false;
      }
    }

    // Check if leaf collides.
    if (!tree1->nodeHasChildren(root1) && bvn2.isLeaf()) {
      assert(tree1->isNodeOccupied(root1));  // it isn't free nor uncertain.
      Box box;
      Transform3f box_tf;
      constructBox(bv1, tf1, box, box_tf);

      int primitive_id = bvn2.primitiveId();
      const Triangle& tri_id = tree2->tri_indices[primitive_id];
      const Vec3f& p1 = tree2->vertices[tri_id[0]];
      const Vec3f& p2 = tree2->vertices[tri_id[1]];
      const Vec3f& p3 = tree2->vertices[tri_id[2]];

      Vec3f c1, c2, normal;
      FCL_REAL distance;

      bool collision = solver->shapeTriangleInteraction(
          box, box_tf, p1, p2, p3, tf2, distance, c1, c2, normal);
      FCL_REAL distToCollision = distance - crequest->security_margin;

      if (cresult->numContacts() < crequest->num_max_contacts) {
        if (collision) {
          cresult->addContact(Contact(tree1, tree2,
                                      (int)(root1 - tree1->getRoot()),
                                      primitive_id, c1, normal, -distance));
        } else if (distToCollision < 0) {
          cresult->addContact(Contact(
              tree1, tree2, (int)(root1 - tree1->getRoot()), primitive_id,
              .5 * (c1 + c2), (c2 - c1).normalized(), -distance));
        }
      }
      internal::updateDistanceLowerBoundFromLeaf(*crequest, *cresult,
                                                 distToCollision, c1, c2);

      return crequest->isSatisfied(*cresult);
    }

    // Determine which tree to traverse first.
    if (bvn2.isLeaf() ||
        (tree1->nodeHasChildren(root1) && (bv1.size() > bvn2.bv.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree1->nodeChildExists(root1, i)) {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          if (OcTreeMeshIntersectRecurse(tree1, child, child_bv, tree2, root2,
                                         tf1, tf2))
            return true;
        }
      }
    } else {
      if (OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2,
                                     (unsigned int)bvn2.leftChild(), tf1, tf2))
        return true;

      if (OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2,
                                     (unsigned int)bvn2.rightChild(), tf1, tf2))
        return true;
    }

    return false;
  }

  bool OcTreeDistanceRecurse(const OcTree* tree1,
                             const OcTree::OcTreeNode* root1, const AABB& bv1,
                             const OcTree* tree2,
                             const OcTree::OcTreeNode* root2, const AABB& bv2,
                             const Transform3f& tf1,
                             const Transform3f& tf2) const {
    if (!tree1->nodeHasChildren(root1) && !tree2->nodeHasChildren(root2)) {
      if (tree1->isNodeOccupied(root1) && tree2->isNodeOccupied(root2)) {
        Box box1, box2;
        Transform3f box1_tf, box2_tf;
        constructBox(bv1, tf1, box1, box1_tf);
        constructBox(bv2, tf2, box2, box2_tf);

        FCL_REAL dist;
        Vec3f closest_p1, closest_p2, normal;
        solver->shapeDistance(box1, box1_tf, box2, box2_tf, dist, closest_p1,
                              closest_p2, normal);

        dresult->update(dist, tree1, tree2, (int)(root1 - tree1->getRoot()),
                        (int)(root2 - tree2->getRoot()), closest_p1, closest_p2,
                        normal);

        return drequest->isSatisfied(*dresult);
      } else
        return false;
    }

    if (!tree1->isNodeOccupied(root1) || !tree2->isNodeOccupied(root2))
      return false;

    if (!tree2->nodeHasChildren(root2) ||
        (tree1->nodeHasChildren(root1) && (bv1.size() > bv2.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree1->nodeChildExists(root1, i)) {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          FCL_REAL d;
          AABB aabb1, aabb2;
          convertBV(bv1, tf1, aabb1);
          convertBV(bv2, tf2, aabb2);
          d = aabb1.distance(aabb2);

          if (d < dresult->min_distance) {
            if (OcTreeDistanceRecurse(tree1, child, child_bv, tree2, root2, bv2,
                                      tf1, tf2))
              return true;
          }
        }
      }
    } else {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree2->nodeChildExists(root2, i)) {
          const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
          AABB child_bv;
          computeChildBV(bv2, i, child_bv);

          FCL_REAL d;
          AABB aabb1, aabb2;
          convertBV(bv1, tf1, aabb1);
          convertBV(bv2, tf2, aabb2);
          d = aabb1.distance(aabb2);

          if (d < dresult->min_distance) {
            if (OcTreeDistanceRecurse(tree1, root1, bv1, tree2, child, child_bv,
                                      tf1, tf2))
              return true;
          }
        }
      }
    }

    return false;
  }

  bool OcTreeIntersectRecurse(const OcTree* tree1,
                              const OcTree::OcTreeNode* root1, const AABB& bv1,
                              const OcTree* tree2,
                              const OcTree::OcTreeNode* root2, const AABB& bv2,
                              const Transform3f& tf1,
                              const Transform3f& tf2) const {
    // Empty OcTree is considered free.
    if (!root1) return false;
    if (!root2) return false;

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least one of the nodes is free; OR
    ///           2) (two uncertain nodes OR one node occupied and one node
    ///           uncertain) AND cost not required
    if (tree1->isNodeFree(root1) || tree2->isNodeFree(root2))
      return false;
    else if ((tree1->isNodeUncertain(root1) || tree2->isNodeUncertain(root2)))
      return false;

    bool bothAreLeaves =
        (!tree1->nodeHasChildren(root1) && !tree2->nodeHasChildren(root2));
    if (!bothAreLeaves || !crequest->enable_contact) {
      OBB obb1, obb2;
      convertBV(bv1, tf1, obb1);
      convertBV(bv2, tf2, obb2);
      FCL_REAL sqrDistLowerBound;
      if (!obb1.overlap(obb2, *crequest, sqrDistLowerBound)) {
        if (cresult->distance_lower_bound > 0 &&
            sqrDistLowerBound <
                cresult->distance_lower_bound * cresult->distance_lower_bound)
          cresult->distance_lower_bound =
              sqrt(sqrDistLowerBound) - crequest->security_margin;
        return false;
      }
      if (!crequest->enable_contact) {  // Overlap
        if (cresult->numContacts() < crequest->num_max_contacts)
          cresult->addContact(
              Contact(tree1, tree2, static_cast<int>(root1 - tree1->getRoot()),
                      static_cast<int>(root2 - tree2->getRoot())));
        return crequest->isSatisfied(*cresult);
      }
    }

    // Both node are leaves
    if (bothAreLeaves) {
      assert(tree1->isNodeOccupied(root1) && tree2->isNodeOccupied(root2));

      Box box1, box2;
      Transform3f box1_tf, box2_tf;
      constructBox(bv1, tf1, box1, box1_tf);
      constructBox(bv2, tf2, box2, box2_tf);

      FCL_REAL distance;
      Vec3f c1, c2, normal;
      bool collision = solver->shapeDistance(box1, box1_tf, box2, box2_tf,
                                             distance, c1, c2, normal);
      FCL_REAL distToCollision = distance - crequest->security_margin;

      if (cresult->numContacts() < crequest->num_max_contacts) {
        if (collision)
          cresult->addContact(
              Contact(tree1, tree2, static_cast<int>(root1 - tree1->getRoot()),
                      static_cast<int>(root2 - tree2->getRoot()), c1, normal,
                      -distance));
        else if (distToCollision <= 0)
          cresult->addContact(
              Contact(tree1, tree2, static_cast<int>(root1 - tree1->getRoot()),
                      static_cast<int>(root2 - tree2->getRoot()),
                      .5 * (c1 + c2), (c2 - c1).normalized(), -distance));
      }
      internal::updateDistanceLowerBoundFromLeaf(*crequest, *cresult,
                                                 distToCollision, c1, c2);

      return crequest->isSatisfied(*cresult);
    }

    // Determine which tree to traverse first.
    if (!tree2->nodeHasChildren(root2) ||
        (tree1->nodeHasChildren(root1) && (bv1.size() > bv2.size()))) {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree1->nodeChildExists(root1, i)) {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          if (OcTreeIntersectRecurse(tree1, child, child_bv, tree2, root2, bv2,
                                     tf1, tf2))
            return true;
        }
      }
    } else {
      for (unsigned int i = 0; i < 8; ++i) {
        if (tree2->nodeChildExists(root2, i)) {
          const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
          AABB child_bv;
          computeChildBV(bv2, i, child_bv);

          if (OcTreeIntersectRecurse(tree1, root1, bv1, tree2, child, child_bv,
                                     tf1, tf2))
            return true;
        }
      }
    }

    return false;
  }
};

/// @addtogroup Traversal_For_Collision
/// @{

/// @brief Traversal node for octree collision
class HPP_FCL_DLLAPI OcTreeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  OcTreeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned, unsigned, FCL_REAL&) const { return false; }

  void leafCollides(unsigned, unsigned, FCL_REAL& sqrDistLowerBound) const {
    otsolver->OcTreeIntersect(model1, model2, tf1, tf2, request, *result);
    sqrDistLowerBound = std::max((FCL_REAL)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const OcTree* model1;
  const OcTree* model2;

  Transform3f tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for shape-octree collision
template <typename S>
class HPP_FCL_DLLAPI ShapeOcTreeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  ShapeOcTreeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, FCL_REAL&) const {
    return false;
  }

  void leafCollides(unsigned int, unsigned int,
                    FCL_REAL& sqrDistLowerBound) const {
    otsolver->OcTreeShapeIntersect(model2, *model1, tf2, tf1, request, *result);
    sqrDistLowerBound = std::max((FCL_REAL)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const S* model1;
  const OcTree* model2;

  Transform3f tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-shape collision

template <typename S>
class HPP_FCL_DLLAPI OcTreeShapeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  OcTreeShapeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, fcl::FCL_REAL&) const {
    return false;
  }

  void leafCollides(unsigned int, unsigned int,
                    FCL_REAL& sqrDistLowerBound) const {
    otsolver->OcTreeShapeIntersect(model1, *model2, tf1, tf2, request, *result);
    sqrDistLowerBound = std::max((FCL_REAL)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const OcTree* model1;
  const S* model2;

  Transform3f tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for mesh-octree collision
template <typename BV>
class HPP_FCL_DLLAPI MeshOcTreeCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  MeshOcTreeCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, FCL_REAL&) const {
    return false;
  }

  void leafCollides(unsigned int, unsigned int,
                    FCL_REAL& sqrDistLowerBound) const {
    otsolver->OcTreeMeshIntersect(model2, model1, tf2, tf1, request, *result);
    sqrDistLowerBound = std::max((FCL_REAL)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const BVHModel<BV>* model1;
  const OcTree* model2;

  Transform3f tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-mesh collision
template <typename BV>
class HPP_FCL_DLLAPI OcTreeMeshCollisionTraversalNode
    : public CollisionTraversalNodeBase {
 public:
  OcTreeMeshCollisionTraversalNode(const CollisionRequest& request)
      : CollisionTraversalNodeBase(request) {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVDisjoints(unsigned int, unsigned int, FCL_REAL&) const {
    return false;
  }

  void leafCollides(unsigned int, unsigned int,
                    FCL_REAL& sqrDistLowerBound) const {
    std::cout << "leafCollides" << std::endl;
    otsolver->OcTreeMeshIntersect(model1, model2, tf1, tf2, request, *result);
    sqrDistLowerBound = std::max((FCL_REAL)0, result->distance_lower_bound);
    sqrDistLowerBound *= sqrDistLowerBound;
  }

  const OcTree* model1;
  const BVHModel<BV>* model2;

  Transform3f tf1, tf2;

  const OcTreeSolver* otsolver;
};

/// @}

/// @addtogroup Traversal_For_Distance
/// @{

/// @brief Traversal node for octree distance
class HPP_FCL_DLLAPI OcTreeDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  OcTreeDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  FCL_REAL BVDistanceLowerBound(unsigned, unsigned) const { return -1; }

  bool BVDistanceLowerBound(unsigned, unsigned, FCL_REAL&) const {
    return false;
  }

  void leafComputeDistance(unsigned, unsigned int) const {
    otsolver->OcTreeDistance(model1, model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const OcTree* model2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for shape-octree distance
template <typename Shape>
class HPP_FCL_DLLAPI ShapeOcTreeDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  ShapeOcTreeDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  FCL_REAL BVDistanceLowerBound(unsigned int, unsigned int) const { return -1; }

  void leafComputeDistance(unsigned int, unsigned int) const {
    otsolver->OcTreeShapeDistance(model2, *model1, tf2, tf1, request, *result);
  }

  const Shape* model1;
  const OcTree* model2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-shape distance
template <typename Shape>
class HPP_FCL_DLLAPI OcTreeShapeDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  OcTreeShapeDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  FCL_REAL BVDistanceLowerBound(unsigned int, unsigned int) const { return -1; }

  void leafComputeDistance(unsigned int, unsigned int) const {
    otsolver->OcTreeShapeDistance(model1, *model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const Shape* model2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for mesh-octree distance
template <typename BV>
class HPP_FCL_DLLAPI MeshOcTreeDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  MeshOcTreeDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  FCL_REAL BVDistanceLowerBound(unsigned int, unsigned int) const { return -1; }

  void leafComputeDistance(unsigned int, unsigned int) const {
    otsolver->OcTreeMeshDistance(model2, model1, tf2, tf1, request, *result);
  }

  const BVHModel<BV>* model1;
  const OcTree* model2;

  const OcTreeSolver* otsolver;
};

/// @brief Traversal node for octree-mesh distance
template <typename BV>
class HPP_FCL_DLLAPI OcTreeMeshDistanceTraversalNode
    : public DistanceTraversalNodeBase {
 public:
  OcTreeMeshDistanceTraversalNode() {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  FCL_REAL BVDistanceLowerBound(unsigned int, unsigned int) const { return -1; }

  void leafComputeDistance(unsigned int, unsigned int) const {
    otsolver->OcTreeMeshDistance(model1, model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const BVHModel<BV>* model2;

  const OcTreeSolver* otsolver;
};

/// @}

}  // namespace fcl

}  // namespace hpp

/// @endcond

#endif
