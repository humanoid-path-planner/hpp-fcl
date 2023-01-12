#define BOOST_TEST_MODULE BOX_BOX_COLLISION
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/internal/tools.h>

#include "utility.h"

using hpp::fcl::Box;
using hpp::fcl::collide;
using hpp::fcl::CollisionRequest;
using hpp::fcl::CollisionResult;
using hpp::fcl::ComputeCollision;
using hpp::fcl::FCL_REAL;
using hpp::fcl::Transform3f;
using hpp::fcl::Vec3f;

BOOST_AUTO_TEST_CASE(box_box_collision) {
  // Define boxes
  Box shape1(1, 1, 1);
  Box shape2(1, 1, 1);

  // Define transforms
  Transform3f T1 = Transform3f::Identity();
  Transform3f T2 = Transform3f::Identity();

  // Compute collision
  CollisionRequest req;
  req.enable_cached_gjk_guess = true;
  req.distance_upper_bound = 1e-6;
  CollisionResult res;
  ComputeCollision collide_functor(&shape1, &shape2);

  T1.setTranslation(Vec3f(0, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&shape1, T1, &shape2, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  T1.setTranslation(Vec3f(2, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&shape1, T1, &shape2, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);
}
