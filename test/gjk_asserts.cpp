#include <boost/test/included/unit_test.hpp>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/collision.h>

using namespace hpp::fcl;

inline double DegToRad(const double& deg)
{
	static double degToRad = M_PI / 180.;
	return deg * degToRad;
}
std::vector<Vec3f> dirs{
		Vec3f::UnitZ(),
		-Vec3f::UnitZ(),
		Vec3f::UnitY(),
		-Vec3f::UnitY(),
		Vec3f::UnitX(),
		-Vec3f::UnitX()};

inline void CreateSphereMesh(BVHModel<OBBRSS>& model, const double& radius)
{
	int polarSteps{32};
	int azimuthSteps{32};

	const float PI = static_cast<float>(M_PI);

	const float polarStep = PI / (polarSteps - 1);
	const float azimuthStep = 2.0f * PI / (azimuthSteps - 1);
	std::vector<Vec3f> vertices;
	std::vector<Triangle> triangles;

	for (int p = 0; p < polarSteps; p++)
	{
		for (int a = 0; a < azimuthSteps; a++)
		{
			const float x = std::sin(p * polarStep) * std::cos(a * azimuthStep);
			const float y = std::sin(p * polarStep) * std::sin(a * azimuthStep);
			const float z = std::cos(p * polarStep);
			vertices.emplace_back(radius * x, radius * y, radius * z);
		}
	}

	for (int p = 0; p < polarSteps - 1; p++)
	{
		for (int a = 0; a < azimuthSteps - 1; a++)
		{
			uint32_t p0 = p * azimuthSteps + a;
			uint32_t p1 = p * azimuthSteps + (a + 1);
			uint32_t p2 = (p + 1) * azimuthSteps + (a + 1);
			uint32_t p3 = (p + 1) * azimuthSteps + a;
			triangles.emplace_back(p0, p2, p1);
			triangles.emplace_back(p0, p3, p2);
		}
	}
	model.beginModel();
	model.addSubModel(vertices, triangles);
	model.endModel();
}

BOOST_AUTO_TEST_SUITE(TestHppFclCore)

BOOST_AUTO_TEST_CASE(TestSpheres)
{

	BVHModel<OBBRSS> sphere1{};
	BVHModel<OBBRSS> sphere2{};

	CreateSphereMesh(sphere1, 1.);
	CreateSphereMesh(sphere2, 2.);

	CollisionRequest request(CONTACT | DISTANCE_LOWER_BOUND, 1);

	ComputeCollision compute(&sphere2, &sphere1);

	Transform3f sphere1Tf{};
	Transform3f sphere2Tf{};

	for (int i = 0; i < 360; i += 1)
	{
		for (int j = 0; j < 180; j += 1)
		{
			continue;
			/// assertion: src/narrowphase/gjk.cpp:331
			if ((i == 5 && j == 48) || (i == 64 && j == 151) || (i == 98 && j == 47)
				|| (i == 355 && j == 48))
				continue;
			/// assertion: src/narrowphase/gjk.cpp:1263
			if ((i == 86 && j == 52) || (i == 89 && j == 17) || (i == 89 && j == 58)
				|| (i == 89 && j == 145))
				continue;

			//			std::cout << "i: " << i << ", j: " << j << std::endl;
			sphere2Tf.setQuatRotation(
					Eigen::AngleAxis<double>(DegToRad(i), Vec3f::UnitZ())
					* Eigen::AngleAxis<double>(DegToRad(j), Vec3f::UnitY()));
			for (const Vec3f dir : dirs)
			{
				sphere2Tf.setTranslation(dir);
				CollisionResult result;

				BOOST_CHECK_NO_THROW(compute(sphere2Tf, sphere1Tf, request, result));
			}
		}
	}
}

BOOST_AUTO_TEST_CASE(TestTriangles)
{

	std::vector<Vec3f> triVertices{Vec3f(1, 0, 0), Vec3f(1, 1, 0), Vec3f(0, 1, 0)};
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
	for (int i = 0; i < 360; i += 30)
	{
		for (int j = 0; j < 180; j += 30)
		{
			for (int k = 0; k < 180; k += 30)
			{
				tri1Tf.setQuatRotation(
						Eigen::AngleAxis<double>(0., Vec3f::UnitZ())
						* Eigen::AngleAxis<double>(DegToRad(k), Vec3f::UnitY()));
				tri2Tf.setQuatRotation(
						Eigen::AngleAxis<double>(DegToRad(i), Vec3f::UnitZ())
						* Eigen::AngleAxis<double>(DegToRad(j), Vec3f::UnitY()));
				CollisionResult result;

				/// assertion: src/collision_node.cpp:58
				//				BOOST_CHECK_NO_THROW(compute(tri2Tf, tri1Tf, request, result));
			}
		}
	}
}

BOOST_AUTO_TEST_SUITE_END()