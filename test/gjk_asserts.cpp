#define BOOST_TEST_MODULE FCL_GJK_ASSERTS

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

void CreateSphereMesh(BVHModel<OBBRSS>& model, const double& radius) {
  size_t polarSteps{32};
  size_t azimuthSteps{32};

  const float PI = static_cast<float>(pi);

  const float polarStep = PI / (float)(polarSteps - 1);
  const float azimuthStep = 2.0f * PI / (float)(azimuthSteps - 1);
  std::vector<Vec3f> vertices;
  std::vector<Triangle> triangles;

  for (size_t p = 0; p < polarSteps; ++p) {
    for (size_t a = 0; a < azimuthSteps; ++a) {
      const float x =
          std::sin((float)p * polarStep) * std::cos((float)a * azimuthStep);
      const float y =
          std::sin((float)p * polarStep) * std::sin((float)a * azimuthStep);
      const float z = std::cos((float)p * polarStep);
      vertices.emplace_back(radius * x, radius * y, radius * z);
    }
  }

  for (size_t p = 0; p < polarSteps - 1; ++p) {
    for (size_t a = 0; a < azimuthSteps - 1; ++a) {
      size_t p0 = p * azimuthSteps + a;
      size_t p1 = p * azimuthSteps + (a + 1);
      size_t p2 = (p + 1) * azimuthSteps + (a + 1);
      size_t p3 = (p + 1) * azimuthSteps + a;
      triangles.emplace_back(p0, p2, p1);
      triangles.emplace_back(p0, p3, p2);
    }
  }
  model.beginModel();
  model.addSubModel(vertices, triangles);
  model.endModel();
}

BOOST_AUTO_TEST_CASE(TestSpheres) {
  BVHModel<OBBRSS> sphere1{};
  BVHModel<OBBRSS> sphere2{};

  CreateSphereMesh(sphere1, 1.);
  CreateSphereMesh(sphere2, 2.);

  CollisionRequest request(CONTACT | DISTANCE_LOWER_BOUND, 1);

  ComputeCollision compute(&sphere2, &sphere1);

  Transform3f sphere1Tf = Transform3f::Identity();
  Transform3f sphere2Tf = Transform3f::Identity();

  for (int i = 0; i < 360; ++i) {
    for (int j = 0; j < 180; ++j) {
      if (
          /// assertion: src/narrowphase/gjk.cpp:331
          (i == 5 && j == 48) || (i == 64 && j == 151) ||
          (i == 98 && j == 47) || (i == 355 && j == 48) ||
          /// assertion: src/narrowphase/gjk.cpp:1263
          (i == 86 && j == 52) || (i == 89 && j == 17) ||
          (i == 89 && j == 58) || (i == 89 && j == 145)) {
        sphere2Tf.setQuatRotation(
            Eigen::AngleAxis<double>(DegToRad(i), Vec3f::UnitZ()) *
            Eigen::AngleAxis<double>(DegToRad(j), Vec3f::UnitY()));
        for (const Vec3f& dir : dirs) {
          sphere2Tf.setTranslation(dir);
          CollisionResult result;

          BOOST_CHECK_NO_THROW(compute(sphere2Tf, sphere1Tf, request, result));
        }
      }
    }
  }
}
