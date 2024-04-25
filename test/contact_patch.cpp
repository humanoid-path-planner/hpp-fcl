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

/** \author Louis Montaut */

#define BOOST_TEST_MODULE FCL_CONTACT_PATCH
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/contact_patch.h>

#include "utility.h"

using namespace hpp::fcl;

BOOST_AUTO_TEST_CASE(box_box_no_collision) {
  const FCL_REAL halfside = 1;
  const Box box1(halfside, halfside, halfside);
  const Box box2(halfside, halfside, halfside);

  const Transform3f tf1;
  Transform3f tf2;
  // set translation to separate the shapes
  const FCL_REAL offset = 0.1;
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
