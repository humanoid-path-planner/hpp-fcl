/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021 INRIA.
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

namespace utf = boost::unit_test::framework;

using namespace hpp::fcl;

template<typename T>
bool check(const T & value, const T & other)
{
  return value == other;
}

template<typename T>
void test_serialization(const T & value, T & other_value)
{
  const std::string tmp_dir(boost::archive::tmpdir());
  const std::string txt_filename = tmp_dir + "file.txt";
  const std::string bin_filename = tmp_dir + "file.bin";

  // TXT
  {
    std::ofstream ofs(txt_filename.c_str());
  
    boost::archive::text_oarchive oa(ofs);
    oa << value;
  }
  BOOST_CHECK(check(value,value));
  
  {
    std::ifstream ifs(txt_filename.c_str());
    boost::archive::text_iarchive ia(ifs);

    ia >> other_value;
  }
  BOOST_CHECK(check(value,other_value));
  
  // BIN
  {
    std::ofstream ofs(bin_filename.c_str(), std::ios::binary);
    boost::archive::binary_oarchive oa(ofs);
    oa << value;
  }
  BOOST_CHECK(check(value,value));
  
  {
    std::ifstream ifs(bin_filename.c_str(), std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);

    ia >> other_value;
  }
  BOOST_CHECK(check(value,other_value));
}

template<typename T>
void test_serialization(const T & value)
{
  T other_value;
  test_serialization(value,other_value);
}

BOOST_AUTO_TEST_CASE(test_aabb)
{
  AABB aabb(-Vec3f::Ones(),Vec3f::Ones());
  test_serialization(aabb);
}

BOOST_AUTO_TEST_CASE(test_collision_data)
{
  Contact contact(NULL, NULL, 1, 2, Vec3f::Ones(), Vec3f::Zero(), -10.);
  test_serialization(contact);
  
  CollisionRequest collision_request(CONTACT,10);
  test_serialization(collision_request);
  
  CollisionResult collision_result;
  collision_result.addContact(contact);
  collision_result.addContact(contact);
  collision_result.distance_lower_bound = 0.1;
  test_serialization(collision_result);
  
  DistanceRequest distance_request(true,1.,2.);
  test_serialization(distance_request);
  
  DistanceResult distance_result;
  distance_result.normal.setOnes();
  distance_result.nearest_points[0].setRandom();
  distance_result.nearest_points[1].setRandom();
  test_serialization(distance_result);
}

BOOST_AUTO_TEST_CASE(test_BVHModel)
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  BVHModel<OBBRSS> m1,m2;

  m1.beginModel();
  m1.addSubModel(p1, t1);
  m1.endModel();
  BOOST_CHECK(m1 == m1);

  m2.beginModel();
  m2.addSubModel(p2, t2);
  m2.endModel();
  BOOST_CHECK(m2 == m2);
  BOOST_CHECK(m1 != m2);
  
  // Test CollisionGeometry
  {
    CollisionGeometry & m1_cg = static_cast<CollisionGeometry &>(m1);
    BVHModel<OBBRSS> m1_copy;
    CollisionGeometry & m1_copy_cg = static_cast<CollisionGeometry &>(m1);
    test_serialization(m1_cg,m1_copy_cg);
  }
  
  // Test BVHModelBase
  {
    BVHModelBase & m1_base = static_cast<BVHModelBase &>(m1);
    BVHModel<OBBRSS> m1_copy;
    BVHModelBase & m1_copy_base = static_cast<BVHModelBase &>(m1);
    test_serialization(m1_base,m1_copy_base);
  }
  
  // Test BVHModel
  {
    BVHModel<OBBRSS> m1_copy;
    test_serialization(m1,m1_copy);
  }
}
