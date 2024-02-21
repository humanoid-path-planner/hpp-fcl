/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-2023 INRIA.
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
#include <hpp/fcl/serialization/transform.h>
#include <hpp/fcl/serialization/geometric_shapes.h>
#include <hpp/fcl/serialization/convex.h>
#include <hpp/fcl/serialization/memory.h>

#ifdef HPP_FCL_HAS_OCTOMAP
#include <hpp/fcl/serialization/octree.h>
#endif

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

template <typename T>
bool check_ptr(const T* value, const T* other) {
  return *value == *other;
}

enum SerializationMode { TXT = 1, XML = 2, BIN = 4, STREAM = 8 };

template <typename T>
void test_serialization(const T* value, T& other_value,
                        const int mode = TXT | XML | BIN | STREAM) {
  test_serialization(*value, other_value, mode);
}

template <typename T,
          bool is_base = std::is_base_of<T, CollisionGeometry>::value>
struct test_pointer_serialization_impl {
  static void run(const T&, T&, const int) {}
};

template <typename T>
struct test_pointer_serialization_impl<T, true> {
  static void run(const T& value, T& other_value, const int mode) {
    const CollisionGeometry* ptr = &value;
    CollisionGeometry* other_ptr = &other_value;

    const boost::filesystem::path tmp_path(boost::archive::tmpdir());
    const boost::filesystem::path txt_path("file.txt");
    const boost::filesystem::path txt_ptr_path("ptr_file.txt");
    const boost::filesystem::path xml_path("file.xml");
    const boost::filesystem::path bin_path("file.bin");
    const boost::filesystem::path txt_filename(tmp_path / txt_path);
    const boost::filesystem::path xml_filename(tmp_path / xml_path);
    const boost::filesystem::path bin_filename(tmp_path / bin_path);

    // TXT
    if (mode & 0x1) {
      {
        std::ofstream ofs(txt_filename.c_str());

        boost::archive::text_oarchive oa(ofs);
        oa << ptr;
      }
      BOOST_CHECK(check(*reinterpret_cast<const CollisionGeometry*>(ptr),
                        *reinterpret_cast<const CollisionGeometry*>(ptr)));

      {
        std::ifstream ifs(txt_filename.c_str());
        boost::archive::text_iarchive ia(ifs);

        ia >> other_ptr;
      }
      BOOST_CHECK(
          check(*reinterpret_cast<const CollisionGeometry*>(ptr),
                *reinterpret_cast<const CollisionGeometry*>(other_ptr)));
    }
  }
};

template <typename T>
void test_pointer_serialization(const T& value, T& other_value,
                                const int mode = TXT | XML | BIN | STREAM) {
  test_pointer_serialization_impl<T>::run(value, other_value, mode);
}

template <typename T>
void test_serialization(const T& value, T& other_value,
                        const int mode = TXT | XML | BIN | STREAM) {
  const boost::filesystem::path tmp_path(boost::archive::tmpdir());
  const boost::filesystem::path txt_path("file.txt");
  const boost::filesystem::path txt_ptr_path("ptr_file.txt");
  const boost::filesystem::path xml_path("file.xml");
  const boost::filesystem::path bin_path("file.bin");
  const boost::filesystem::path txt_filename(tmp_path / txt_path);
  const boost::filesystem::path xml_filename(tmp_path / xml_path);
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

  // XML
  if (mode & 0x2) {
    {
      std::ofstream ofs(xml_filename.c_str());
      boost::archive::xml_oarchive oa(ofs);
      oa << boost::serialization::make_nvp("value", value);
    }
    BOOST_CHECK(check(value, value));

    {
      std::ifstream ifs(xml_filename.c_str());
      boost::archive::xml_iarchive ia(ifs, boost::archive::no_codecvt);

      ia >> boost::serialization::make_nvp("value", other_value);
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

  // Test std::shared_ptr<T>
  {
    const boost::filesystem::path txt_ptr_filename(tmp_path / txt_ptr_path);
    std::shared_ptr<T> ptr = std::make_shared<T>(value);

    {
      std::ofstream ofs(txt_ptr_filename.c_str());

      boost::archive::text_oarchive oa(ofs);
      oa << ptr;
    }
    BOOST_CHECK(check_ptr(ptr.get(), ptr.get()));

    std::shared_ptr<T> other_ptr = nullptr;
    {
      std::ifstream ifs(txt_ptr_filename.c_str());
      boost::archive::text_iarchive ia(ifs);

      ia >> other_ptr;
    }
    BOOST_CHECK(check_ptr(ptr.get(), other_ptr.get()));
  }

  test_pointer_serialization(value, other_value);
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
  collision_result.normal.setOnes();
  collision_result.nearest_points[0].setRandom();
  collision_result.nearest_points[1].setRandom();
  test_serialization(collision_result);

  DistanceRequest distance_request(true, 1., 2.);
  test_serialization(distance_request);

  DistanceResult distance_result;
  distance_result.normal.setOnes();
  distance_result.nearest_points[0].setRandom();
  distance_result.nearest_points[1].setRandom();
  test_serialization(distance_result);
}

template <typename T>
void checkEqualStdVector(const std::vector<T>& v1, const std::vector<T>& v2) {
  BOOST_CHECK(v1.size() == v2.size());
  if (v1.size() == v2.size()) {
    for (size_t i = 0; i < v1.size(); i++) {
      BOOST_CHECK(v1[i] == v2[i]);
    }
  }
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
  BOOST_CHECK(m1.num_vertices == p1.size());
  BOOST_CHECK(m1.num_tris == t1.size());
  checkEqualStdVector(*m1.vertices, p1);
  checkEqualStdVector(*m1.tri_indices, t1);
  BOOST_CHECK(m1 == m1);

  m2.beginModel();
  m2.addSubModel(p2, t2);
  m2.endModel();
  BOOST_CHECK(m2.num_vertices == p2.size());
  BOOST_CHECK(m2.num_tris == t2.size());
  checkEqualStdVector(*m2.vertices, p2);
  checkEqualStdVector(*m2.tri_indices, t2);
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

  // Test std::shared_ptr<CollisionGeometry>
  {
    const boost::filesystem::path tmp_dir(boost::archive::tmpdir());
    const boost::filesystem::path txt_filename = tmp_dir / "file.txt";
    const boost::filesystem::path bin_filename = tmp_dir / "file.bin";
    Convex<Triangle> convex_copy;

    std::shared_ptr<CollisionGeometry> ptr =
        std::make_shared<Convex<Triangle>>(convex);
    BOOST_CHECK(ptr.get());
    {
      std::ofstream ofs(txt_filename.c_str());

      boost::archive::text_oarchive oa(ofs);
      oa << ptr;
    }
    BOOST_CHECK(check(*reinterpret_cast<Convex<Triangle>*>(ptr.get()), convex));

    std::shared_ptr<CollisionGeometry> other_ptr = nullptr;
    BOOST_CHECK(!other_ptr.get());
    {
      std::ifstream ifs(txt_filename.c_str());
      boost::archive::text_iarchive ia(ifs);

      ia >> other_ptr;
    }
    BOOST_CHECK(
        check(convex, *reinterpret_cast<Convex<Triangle>*>(other_ptr.get())));
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

BOOST_AUTO_TEST_CASE(test_transform) {
  Transform3f T;
  T.setQuatRotation(Quaternion3f::UnitRandom());
  T.setTranslation(Vec3f::Random());

  Transform3f T_copy;
  test_serialization(T, T_copy);
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

#ifdef HPP_FCL_HAS_OCTOMAP
BOOST_AUTO_TEST_CASE(test_octree) {
  const FCL_REAL resolution = 1e-2;
  const Matrixx3f points = Matrixx3f::Random(1000, 3);
  OcTreePtr_t octree_ptr = makeOctree(points, resolution);
  const OcTree& octree = *octree_ptr.get();

  const boost::filesystem::path tmp_dir(boost::archive::tmpdir());
  const boost::filesystem::path txt_filename = tmp_dir / "file.txt";
  const boost::filesystem::path bin_filename = tmp_dir / "file.bin";

  {
    std::ofstream ofs(bin_filename.c_str(), std::ios::binary);
    boost::archive::binary_oarchive oa(ofs);
    oa << octree;
  }

  OcTree octree_value(1.);
  {
    std::ifstream ifs(bin_filename.c_str(),
                      std::fstream::binary | std::fstream::in);
    boost::archive::binary_iarchive ia(ifs);

    ia >> octree_value;
  }

  BOOST_CHECK(octree.getTree() == octree.getTree());
  BOOST_CHECK(octree_value.getTree() == octree_value.getTree());
  //  BOOST_CHECK(octree.getTree() == octree_value.getTree());
  BOOST_CHECK(octree.getResolution() == octree_value.getResolution());
  BOOST_CHECK(octree.getTree()->size() == octree_value.getTree()->size());
  BOOST_CHECK(octree.toBoxes().size() == octree_value.toBoxes().size());
  BOOST_CHECK(octree == octree_value);

  OcTree octree_copy(1.);
  test_serialization(octree, octree_copy);
}
#endif

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
