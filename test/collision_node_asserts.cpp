#define BOOST_TEST_MODULE FCL_COLLISION_NODE_ASSERT

#include <boost/test/included/unit_test.hpp>
#include <boost/math/constants/constants.hpp>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/collision.h>

using namespace hpp::fcl;

constexpr FCL_REAL pi = boost::math::constants::pi<FCL_REAL>();

double DegToRad(const double& deg) {
  static double degToRad = pi / 180.;
  return deg * degToRad;
}
std::vector<Vec3f> dirs{Vec3f::UnitZ(),  -Vec3f::UnitZ(), Vec3f::UnitY(),
                        -Vec3f::UnitY(), Vec3f::UnitX(),  -Vec3f::UnitX()};

BOOST_AUTO_TEST_CASE(TestTriangles) {
  std::vector<Vec3f> triVertices{Vec3f(1, 0, 0), Vec3f(1, 1, 0),
                                 Vec3f(0, 1, 0)};
  std::vector<Triangle> triangle{{0, 1, 2}};

  BVHModel<OBBRSS> tri1{};
  BVHModel<OBBRSS> tri2{};

  tri1.beginModel();
  tri1.addSubModel(triVertices, triangle);
  tri1.endModel();

  tri2.beginModel();
  tri2.addSubModel(triVertices, triangle);
  tri2.endModel();

  CollisionRequest request(CONTACT | DISTANCE_LOWER_BOUND, 1);

  ComputeCollision compute(&tri1, &tri2);

  Transform3f tri1Tf{};
  Transform3f tri2Tf{};

  /// check some angles for two triangles
  for (int i = 0; i < 360; i += 30) {
    for (int j = 0; j < 180; j += 30) {
      for (int k = 0; k < 180; k += 30) {
        tri1Tf.setQuatRotation(
            Eigen::AngleAxis<double>(0., Vec3f::UnitZ()) *
            Eigen::AngleAxis<double>(DegToRad(k), Vec3f::UnitY()));
        tri2Tf.setQuatRotation(
            Eigen::AngleAxis<double>(DegToRad(i), Vec3f::UnitZ()) *
            Eigen::AngleAxis<double>(DegToRad(j), Vec3f::UnitY()));
        CollisionResult result;

        /// assertion: src/collision_node.cpp:58
        BOOST_CHECK_NO_THROW(compute(tri2Tf, tri1Tf, request, result));
      }
    }
  }
}
