/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-2022 INRIA.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#define BOOST_TEST_MODULE FCL_SERIALIZATION
#include <fstream>
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/BV/OBBRSS.h>
#include <hpp/fcl/BVH/BVH_model.h>

#include <hpp/fcl/serialization/collision_data.h>
#include <hpp/fcl/serialization/AABB.h>
#include <hpp/fcl/serialization/BVH_model.h>
#include <hpp/fcl/serialization/hfield.h>
#include <hpp/fcl/serialization/geometric_shapes.h>
#include <hpp/fcl/serialization/convex.h>
#include <hpp/fcl/serialization/memory.h>

#include "utility.h"
#include "fcl_resources/config.h"

#include <boost/archive/tmpdir.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>

#include <boost/asio/streambuf.hpp>

namespace utf = boost::unit_test::framework;

using namespace hpp::fcl;

template <typename T>
void saveToBinary(const T& object, boost::asio::streambuf& buffer) {
  boost::archive::binary_oarchive oa(buffer);
  oa & object;
}

template <typename T>
inline void loadFromBinary(T& object, boost::asio::streambuf& buffer) {
  boost::archive::binary_iarchive ia(buffer);
  ia >> object;
}

template <typename T>
bool check(const T& value, const T& other) {
  return value == other;
}

enum SerializationMode { TXT = 1, XML = 2, BIN = 4, STREAM = 8 };

template <typename T>
void test_serialization(const T& value, T& other_value,
                        const int mode = TXT | XML | BIN | STREAM) {
  const boost::filesystem::path tmp_path(boost::archive::tmpdir());
  const boost::filesystem::path txt_path("file.txt");
  const boost::filesystem::path bin_path("file.bin");
  const boost::filesystem::path txt_filename(tmp_path / txt_path);
  const boost::filesystem::path bin_filename(tmp_path / bin_path);

  // TXT
  if (mode & 0x1) {
    {
      std::ofstream ofs(txt_filename.c_str());

      boost::archive::text_oarchive oa(ofs);
      oa << value;
    }
    BOOST_CHECK(check(value, value));

    {
      std::ifstream ifs(txt_filename.c_str());
      boost::archive::text_iarchive ia(ifs);

      ia >> other_value;
    }
    BOOST_CHECK(check(value, other_value));
  }

  // BIN
  if (mode & 0x4) {
    {
      std::ofstream ofs(bin_filename.c_str(), std::ios::binary);
      boost::archive::binary_oarchive oa(ofs);
      oa << value;
    }
    BOOST_CHECK(check(value, value));

    {
      std::ifstream ifs(bin_filename.c_str(), std::ios::binary);
      boost::archive::binary_iarchive ia(ifs);

      ia >> other_value;
    }
    BOOST_CHECK(check(value, other_value));
  }

  // Stream Buffer
  if (mode & 0x8) {
    boost::asio::streambuf buffer;
    saveToBinary(value, buffer);
    BOOST_CHECK(check(value, value));

    loadFromBinary(other_value, buffer);
    BOOST_CHECK(check(value, other_value));
  }
}

template <typename T>
void test_serialization(const T& value,
                        const int mode = TXT | XML | BIN | STREAM) {
  T other_value;
  test_serialization(value, other_value, mode);
}

BOOST_AUTO_TEST_CASE(test_aabb) {
  AABB aabb(-Vec3f::Ones(), Vec3f::Ones());
  test_serialization(aabb);
}

BOOST_AUTO_TEST_CASE(test_collision_data) {
  Contact contact(NULL, NULL, 1, 2, Vec3f::Ones(), Vec3f::Zero(), -10.);
  test_serialization(contact);

  CollisionRequest collision_request(CONTACT, 10);
  test_serialization(collision_request);

  CollisionResult collision_result;
  collision_result.addContact(contact);
  collision_result.addContact(contact);
  collision_result.distance_lower_bound = 0.1;
  test_serialization(collision_result);

  DistanceRequest distance_request(true, 1., 2.);
  test_serialization(distance_request);

  DistanceResult distance_result;
  distance_result.normal.setOnes();
  distance_result.nearest_points[0].setRandom();
  distance_result.nearest_points[1].setRandom();
  test_serialization(distance_result);
}

BOOST_AUTO_TEST_CASE(test_BVHModel) {
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  BVHModel<OBBRSS> m1, m2;

  m1.beginModel();
  m1.addSubModel(p1, t1);
  m1.endModel();
  BOOST_CHECK(m1 == m1);

  m2.beginModel();
  m2.addSubModel(p2, t2);
  m2.endModel();
  BOOST_CHECK(m2 == m2);
  BOOST_CHECK(m1 != m2);

  // Test BVHModel
  {
    BVHModel<OBBRSS> m1_copy;
    test_serialization(m1, m1_copy);
  }
  {
    BVHModel<OBBRSS> m1_copy;
    test_serialization(m1, m1_copy, STREAM);
  }
}

#ifdef HPP_FCL_HAS_QHULL
BOOST_AUTO_TEST_CASE(test_Convex) {
  std::vector<Vec3f> p1;
  std::vector<Triangle> t1;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);

  BVHModel<OBBRSS> m1;

  m1.beginModel();
  m1.addSubModel(p1, t1);
  m1.endModel();

  m1.buildConvexHull(true);

  Convex<Triangle>& convex = static_cast<Convex<Triangle>&>(*m1.convex.get());

  // Test Convex
  {
    Convex<Triangle> convex_copy;
    test_serialization(convex, convex_copy);
  }
}
#endif

BOOST_AUTO_TEST_CASE(test_HeightField) {
  const FCL_REAL min_altitude = -1.;
  const FCL_REAL x_dim = 1., y_dim = 2.;
  const Eigen::DenseIndex nx = 100, ny = 200;
  const MatrixXf heights = MatrixXf::Random(ny, nx);

  HeightField<OBBRSS> hfield(x_dim, y_dim, heights, min_altitude);

  // Test HeightField
  {
    HeightField<OBBRSS> hfield_copy;
    test_serialization(hfield, hfield_copy);
  }
  {
    HeightField<OBBRSS> hfield_copy;
    test_serialization(hfield, hfield_copy, STREAM);
  }
}

BOOST_AUTO_TEST_CASE(test_shapes) {
  {
    TriangleP triangle(Vec3f::UnitX(), Vec3f::UnitY(), Vec3f::UnitZ());
    TriangleP triangle_copy(Vec3f::Random(), Vec3f::Random(), Vec3f::Random());
    test_serialization(triangle, triangle_copy);
  }

  {
    Box box(Vec3f::UnitX()), box_copy(Vec3f::Random());
    test_serialization(box, box_copy);
  }

  {
    Sphere sphere(1.), sphere_copy(2.);
    test_serialization(sphere, sphere_copy);
  }

  {
    Ellipsoid ellipsoid(1., 2., 3.), ellipsoid_copy(0., 0., 0.);
    test_serialization(ellipsoid, ellipsoid_copy);
  }

  {
    Capsule capsule(1., 2.), capsule_copy(10., 10.);
    test_serialization(capsule, capsule_copy);
  }

  {
    Cone cone(1., 2.), cone_copy(10., 10.);
    test_serialization(cone, cone_copy);
  }

  {
    Cylinder cylinder(1., 2.), cylinder_copy(10., 10.);
    test_serialization(cylinder, cylinder_copy);
  }

  {
    Halfspace hs(Vec3f::Random(), 1.), hs_copy(Vec3f::Zero(), 0.);
    test_serialization(hs, hs_copy);
  }

  {
    Plane plane(Vec3f::Random(), 1.), plane_copy(Vec3f::Zero(), 0.);
    test_serialization(plane, plane_copy);
  }
}

BOOST_AUTO_TEST_CASE(test_memory_footprint) {
  Sphere sphere(1.);
  BOOST_CHECK(sizeof(Sphere) == computeMemoryFootprint(sphere));

  std::vector<Vec3f> p1;
  std::vector<Triangle> t1;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);

  BVHModel<OBBRSS> m1;

  m1.beginModel();
  m1.addSubModel(p1, t1);
  m1.endModel();

  std::cout << "computeMemoryFootprint(m1): " << computeMemoryFootprint(m1)
            << std::endl;
  BOOST_CHECK(sizeof(BVHModel<OBBRSS>) < computeMemoryFootprint(m1));
  BOOST_CHECK(static_cast<size_t>(m1.memUsage(false)) ==
              computeMemoryFootprint(m1));
}
