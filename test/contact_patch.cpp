/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
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
 *   * Neither the name of INRIA nor the names of its
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

/** \author Louis Montaut */

#define BOOST_TEST_MODULE FCL_CONTACT_PATCH
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/contact_patch.h>

#include "utility.h"

using namespace hpp::fcl;

BOOST_AUTO_TEST_CASE(box_box_no_collision) {
  const FCL_REAL halfside = 0.5;
  const Box box1(2 * halfside, 2 * halfside, 2 * halfside);
  const Box box2(2 * halfside, 2 * halfside, 2 * halfside);

  const Transform3f tf1;
  Transform3f tf2;
  // set translation to separate the shapes
  const FCL_REAL offset = 0.001;
  tf2.setTranslation(Vec3f(0, 0, 2 * halfside + offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;

  hpp::fcl::collide(&box1, tf1, &box2, tf2, col_req, col_res);

  BOOST_CHECK(!col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);
  hpp::fcl::computeContactPatch(&box1, tf1, &box2, tf2, col_res, patch_req,
                                patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 0);
}

BOOST_AUTO_TEST_CASE(box_box) {
  const FCL_REAL halfside = 0.5;
  const Box box1(2 * halfside, 2 * halfside, 2 * halfside);
  const Box box2(2 * halfside, 2 * halfside, 2 * halfside);

  const Transform3f tf1;
  Transform3f tf2;
  // set translation to have a collision
  const FCL_REAL offset = 0.001;
  tf2.setTranslation(Vec3f(0, 0, 2 * halfside - offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;

  hpp::fcl::collide(&box1, tf1, &box2, tf2, col_req, col_res);

  BOOST_CHECK(col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);
  hpp::fcl::computeContactPatch(&box1, tf1, &box2, tf2, col_res, patch_req,
                                patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);

  if (patch_res.numContactPatches() > 0) {
    const Contact& contact = col_res.getContact(0);

    const size_t expected_size = 4;
    ContactPatch expected(expected_size);
    const FCL_REAL tol = 1e-6;
    const Vec3f& n = contact.normal;
    const FCL_REAL d = contact.penetration_depth;
    EIGEN_VECTOR_IS_APPROX(n, Vec3f(0, 0, 1), tol);
    expected.tfc.setIdentity();
    expected.penetration_depth = contact.penetration_depth;
    const std::array<Vec3f, 4> corners = {
        Vec3f(halfside, halfside, halfside),
        Vec3f(halfside, -halfside, halfside),
        Vec3f(-halfside, -halfside, halfside),
        Vec3f(-halfside, halfside, halfside),
    };
    for (size_t i = 0; i < expected_size; ++i) {
      // Contact point expressed in the local frame of the expected contact
      // patch.
      const Vec3f p = expected.tfc.inverseTransform(corners[i] + (d * n) / 2);
      expected.addContactPoint(p.head(2));
    }

    const ContactPatch& contact_patch = patch_res.contact_patches[0];
    BOOST_CHECK(contact_patch.isSame(expected, tol));
  }
}

BOOST_AUTO_TEST_CASE(halfspace_box) {
  const Halfspace hspace(0, 0, 1, 0);
  const FCL_REAL halfside = 0.5;
  const Box box(2 * halfside, 2 * halfside, 2 * halfside);

  const Transform3f tf1;
  Transform3f tf2;
  // set translation to have a collision
  const FCL_REAL offset = 0.001;
  tf2.setTranslation(Vec3f(0, 0, halfside - offset));

  const size_t num_max_contact = 1;
  const CollisionRequest col_req(CollisionRequestFlag::CONTACT,
                                 num_max_contact);
  CollisionResult col_res;

  hpp::fcl::collide(&hspace, tf1, &box, tf2, col_req, col_res);

  BOOST_CHECK(col_res.isCollision());

  const ContactPatchRequest patch_req;
  ContactPatchResult patch_res(patch_req);
  hpp::fcl::computeContactPatch(&hspace, tf1, &box, tf2, col_res, patch_req,
                                patch_res);
  BOOST_CHECK(patch_res.numContactPatches() == 1);

  if (patch_res.numContactPatches() > 0) {
    const Contact& contact = col_res.getContact(0);
    const FCL_REAL tol = 1e-6;
    EIGEN_VECTOR_IS_APPROX(contact.normal, hspace.n, tol);
    EIGEN_VECTOR_IS_APPROX(hspace.n, Vec3f(0, 0, 1), tol);

    const size_t expected_size = 4;
    ContactPatch expected(expected_size);
    const FCL_REAL d = contact.penetration_depth;
    const Vec3f& n = contact.normal;
    expected.tfc.setIdentity();
    expected.penetration_depth = contact.penetration_depth;
    const std::array<Vec3f, 4> corners = {
        tf2.transform(Vec3f(halfside, halfside, -halfside)),
        tf2.transform(Vec3f(halfside, -halfside, -halfside)),
        tf2.transform(Vec3f(-halfside, -halfside, -halfside)),
        tf2.transform(Vec3f(-halfside, halfside, -halfside)),
    };
    for (size_t i = 0; i < expected_size; ++i) {
      // Contact point expressed in the local frame of the expected contact
      // patch.
      const Vec3f p = expected.tfc.inverseTransform(corners[i] + (d * n) / 2);
      expected.addContactPoint(p.head(2));
    }

    const ContactPatch& contact_patch = patch_res.contact_patches[0];
    BOOST_CHECK(contact_patch.isSame(expected, tol));
  }
}

BOOST_AUTO_TEST_CASE(convex_convex) {
  // TODO(louis)
  const FCL_REAL halfside = 1;
  const Convex<Quadrilateral> box1(buildBox(halfside, halfside, halfside));
  const Convex<Quadrilateral> box2(buildBox(halfside, halfside, halfside));
}
