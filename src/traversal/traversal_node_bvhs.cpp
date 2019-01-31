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


#include <hpp/fcl/traversal/traversal_node_bvhs.h>

namespace hpp
{
namespace fcl
{

namespace details
{
template<typename BV>
static inline void meshDistanceOrientedNodeLeafTesting(int b1, int b2,
                                                       const BVHModel<BV>* model1, const BVHModel<BV>* model2,
                                                       Vec3f* vertices1, Vec3f* vertices2, 
                                                       Triangle* tri_indices1, Triangle* tri_indices2,
                                                       const Matrix3f& R, const Vec3f& T,
                                                       bool enable_statistics,
                                                       int& num_leaf_tests,
                                                       const DistanceRequest&,
                                                       DistanceResult& result)
{
  if(enable_statistics) num_leaf_tests++;

  const BVNode<BV>& node1 = model1->getBV(b1);
  const BVNode<BV>& node2 = model2->getBV(b2);

  int primitive_id1 = node1.primitiveId();
  int primitive_id2 = node2.primitiveId();

  const Triangle& tri_id1 = tri_indices1[primitive_id1];
  const Triangle& tri_id2 = tri_indices2[primitive_id2];

  const Vec3f& t11 = vertices1[tri_id1[0]];
  const Vec3f& t12 = vertices1[tri_id1[1]];
  const Vec3f& t13 = vertices1[tri_id1[2]];

  const Vec3f& t21 = vertices2[tri_id2[0]];
  const Vec3f& t22 = vertices2[tri_id2[1]];
  const Vec3f& t23 = vertices2[tri_id2[2]];

  // nearest point pair
  Vec3f P1, P2, normal;

  FCL_REAL d = sqrt (TriangleDistance::sqrTriDistance
		     (t11, t12, t13, t21, t22, t23, R, T, P1, P2));

  result.update(d, model1, model2, primitive_id1, primitive_id2, P1, P2,
                normal);
}
} // namespace details

MeshCollisionTraversalNodeOBB::MeshCollisionTraversalNodeOBB
(const CollisionRequest& request) :
  MeshCollisionTraversalNode<OBB> (request)
{
  R.setIdentity();
}

bool MeshCollisionTraversalNodeOBB::BVTesting(int b1, int b2) const
{
  if(enable_statistics) num_bv_tests++;
  return !overlap(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv);
}

bool MeshCollisionTraversalNodeOBB::BVTesting
(int b1, int b2, FCL_REAL& sqrDistLowerBound) const
{
  if(enable_statistics) num_bv_tests++;
  return !overlap(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv,
                  request, sqrDistLowerBound);
}

MeshCollisionTraversalNodeRSS::MeshCollisionTraversalNodeRSS
(const CollisionRequest& request) :
  MeshCollisionTraversalNode<RSS> (request)
{
  R.setIdentity();
}

bool MeshCollisionTraversalNodeRSS::BVTesting(int b1, int b2) const
{
  if(enable_statistics) num_bv_tests++;
  return !overlap(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv);
}

bool MeshCollisionTraversalNodeRSS::BVTesting(int b1, int b2,
                                              FCL_REAL& sqrDistLowerBound) const
{
  if(enable_statistics) num_bv_tests++;
  sqrDistLowerBound = 0;
  return !overlap(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv);
}

MeshCollisionTraversalNodekIOS::MeshCollisionTraversalNodekIOS
(const CollisionRequest& request) :
  MeshCollisionTraversalNode<kIOS>(request)
{
  R.setIdentity();
}

bool MeshCollisionTraversalNodekIOS::BVTesting(int b1, int b2) const
{
  if(enable_statistics) num_bv_tests++;
  return !overlap(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv);
}

bool MeshCollisionTraversalNodekIOS::BVTesting
(int b1, int b2, FCL_REAL& sqrDistLowerBound) const
{
  if(enable_statistics) num_bv_tests++;
  sqrDistLowerBound = 0;
  return !overlap(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv);
}

MeshCollisionTraversalNodeOBBRSS::MeshCollisionTraversalNodeOBBRSS
(const CollisionRequest& request) :
  MeshCollisionTraversalNode<OBBRSS> (request)
{
  R.setIdentity();
}

bool MeshCollisionTraversalNodeOBBRSS::BVTesting(int b1, int b2) const
{
  if(enable_statistics) num_bv_tests++;
  return !overlap(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv);
}

  bool MeshCollisionTraversalNodeOBBRSS::BVTesting
  (int b1, int b2, FCL_REAL& sqrDistLowerBound) const
  {
    if(enable_statistics) num_bv_tests++;
    bool res (!overlap(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv,
                       request, sqrDistLowerBound));
    assert (!res || sqrDistLowerBound > 0);
    return res;
  }

namespace details
{

template<typename BV>
static inline void distancePreprocessOrientedNode(const BVHModel<BV>* model1, const BVHModel<BV>* model2,
                                                  const Vec3f* vertices1, Vec3f* vertices2,
                                                  Triangle* tri_indices1, Triangle* tri_indices2,
                                                  int init_tri_id1, int init_tri_id2,
                                                  const Matrix3f& R, const Vec3f& T,
                                                  const DistanceRequest&,
                                                  DistanceResult& result)
{
  const Triangle& init_tri1 = tri_indices1[init_tri_id1];
  const Triangle& init_tri2 = tri_indices2[init_tri_id2];

  Vec3f init_tri1_points[3];
  Vec3f init_tri2_points[3];

  init_tri1_points[0] = vertices1[init_tri1[0]];
  init_tri1_points[1] = vertices1[init_tri1[1]];
  init_tri1_points[2] = vertices1[init_tri1[2]];

  init_tri2_points[0] = vertices2[init_tri2[0]];
  init_tri2_points[1] = vertices2[init_tri2[1]];
  init_tri2_points[2] = vertices2[init_tri2[2]];

  Vec3f p1, p2, normal;
  FCL_REAL distance = sqrt (TriangleDistance::sqrTriDistance
			    (init_tri1_points[0], init_tri1_points[1],
			     init_tri1_points[2], init_tri2_points[0],
			     init_tri2_points[1], init_tri2_points[2],
			     R, T, p1, p2));

  result.update(distance, model1, model2, init_tri_id1, init_tri_id2, p1, p2,
                normal);
}

template<typename BV>
static inline void distancePostprocessOrientedNode(const BVHModel<BV>* model1, const BVHModel<BV>* model2,
                                                   const Transform3f& tf1, const DistanceRequest& request, DistanceResult& result)
{
  /// the points obtained by triDistance are not in world space: both are in object1's local coordinate system, so we need to convert them into the world space.
  if(request.enable_nearest_points && (result.o1 == model1) && (result.o2 == model2))
  {
    result.nearest_points[0] = tf1.transform(result.nearest_points[0]).eval();
    result.nearest_points[1] = tf1.transform(result.nearest_points[1]).eval();
  }
}

}

MeshDistanceTraversalNodeRSS::MeshDistanceTraversalNodeRSS() : MeshDistanceTraversalNode<RSS>()
{
  R.setIdentity();
}

void MeshDistanceTraversalNodeRSS::preprocess()
{
  details::distancePreprocessOrientedNode(model1, model2, vertices1, vertices2, tri_indices1, tri_indices2, 0, 0, R, T, request, *result);
}

void MeshDistanceTraversalNodeRSS::postprocess()
{
  details::distancePostprocessOrientedNode(model1, model2, tf1, request, *result);
}

FCL_REAL MeshDistanceTraversalNodeRSS::BVTesting(int b1, int b2) const
{
  if(enable_statistics) num_bv_tests++;
  return distance(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv);
}

void MeshDistanceTraversalNodeRSS::leafTesting(int b1, int b2) const
{
  details::meshDistanceOrientedNodeLeafTesting(b1, b2, model1, model2, vertices1, vertices2, tri_indices1, tri_indices2, 
                                               R, T, enable_statistics, num_leaf_tests, 
                                               request, *result);
}

MeshDistanceTraversalNodekIOS::MeshDistanceTraversalNodekIOS() : MeshDistanceTraversalNode<kIOS>()
{
  R.setIdentity();
}

void MeshDistanceTraversalNodekIOS::preprocess()
{
  details::distancePreprocessOrientedNode(model1, model2, vertices1, vertices2, tri_indices1, tri_indices2, 0, 0, R, T, request, *result);
}

void MeshDistanceTraversalNodekIOS::postprocess()
{
  details::distancePostprocessOrientedNode(model1, model2, tf1, request, *result);
}

FCL_REAL MeshDistanceTraversalNodekIOS::BVTesting(int b1, int b2) const
{
  if(enable_statistics) num_bv_tests++;
  return distance(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv);
}

void MeshDistanceTraversalNodekIOS::leafTesting(int b1, int b2) const
{
  details::meshDistanceOrientedNodeLeafTesting(b1, b2, model1, model2, vertices1, vertices2, tri_indices1, tri_indices2, 
                                               R, T, enable_statistics, num_leaf_tests, 
                                               request, *result);
}

MeshDistanceTraversalNodeOBBRSS::MeshDistanceTraversalNodeOBBRSS() : MeshDistanceTraversalNode<OBBRSS>()
{
  R.setIdentity();
}

void MeshDistanceTraversalNodeOBBRSS::preprocess()
{
  details::distancePreprocessOrientedNode(model1, model2, vertices1, vertices2, tri_indices1, tri_indices2, 0, 0, R, T, request, *result);
}

void MeshDistanceTraversalNodeOBBRSS::postprocess()
{
  details::distancePostprocessOrientedNode(model1, model2, tf1, request, *result);
}

FCL_REAL MeshDistanceTraversalNodeOBBRSS::BVTesting(int b1, int b2) const
{
  if(enable_statistics) num_bv_tests++;
  return distance(R, T, model1->getBV(b1).bv, model2->getBV(b2).bv);
}

void MeshDistanceTraversalNodeOBBRSS::leafTesting(int b1, int b2) const
{
  details::meshDistanceOrientedNodeLeafTesting(b1, b2, model1, model2, vertices1, vertices2, tri_indices1, tri_indices2, 
                                               R, T, enable_statistics, num_leaf_tests, 
                                               request, *result);
}

}

} // namespace hpp
