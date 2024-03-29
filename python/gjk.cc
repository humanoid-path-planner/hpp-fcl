//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2020 CNRS-LAAS
//  Author: Joseph Mirabel
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of CNRS-LAAS. nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#include <eigenpy/eigenpy.hpp>

#include "fcl.hh"

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/narrowphase/gjk.h>

#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#include "doxygen_autodoc/hpp/fcl/narrowphase/gjk.h"
#endif

using namespace boost::python;
using namespace hpp::fcl;
using hpp::fcl::details::EPA;
using hpp::fcl::details::GJK;
using hpp::fcl::details::MinkowskiDiff;

void exposeGJK() {
  if (!eigenpy::register_symbolic_link_to_registered_type<GJK::Status>()) {
    enum_<GJK::Status>("GJKStatus")
        .value("Failed", GJK::Status::Failed)
        .value("DidNotRun", GJK::Status::DidNotRun)
        .value("NoCollision", GJK::Status::NoCollision)
        .value("NoCollisionEarlyStopped", GJK::Status::NoCollisionEarlyStopped)
        .value("CollisionWithPenetrationInformation",
               GJK::Status::CollisionWithPenetrationInformation)
        .value("Collision", GJK::Status::Collision)
        .export_values();
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<MinkowskiDiff>()) {
    class_<MinkowskiDiff>("MinkowskiDiff", doxygen::class_doc<MinkowskiDiff>(),
                          no_init)
        .def(doxygen::visitor::init<MinkowskiDiff>())
        .def("set",
             static_cast<void (MinkowskiDiff::*)(
                 const ShapeBase*, const ShapeBase*)>(&MinkowskiDiff::set),
             doxygen::member_func_doc(
                 static_cast<void (MinkowskiDiff::*)(
                     const ShapeBase*, const ShapeBase*)>(&MinkowskiDiff::set)))
        .def("set",
             static_cast<void (MinkowskiDiff::*)(
                 const ShapeBase*, const ShapeBase*, const Transform3f& tf0,
                 const Transform3f& tf1)>(&MinkowskiDiff::set),
             doxygen::member_func_doc(
                 static_cast<void (MinkowskiDiff::*)(
                     const ShapeBase*, const ShapeBase*, const Transform3f& tf0,
                     const Transform3f& tf1)>(&MinkowskiDiff::set)))
        .DEF_CLASS_FUNC(MinkowskiDiff, support0)
        .DEF_CLASS_FUNC(MinkowskiDiff, support1)
        .DEF_CLASS_FUNC(MinkowskiDiff, support)
        .DEF_RW_CLASS_ATTRIB(MinkowskiDiff, inflation)
        .DEF_RW_CLASS_ATTRIB(MinkowskiDiff, normalize_support_direction);
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<GJKVariant>()) {
    enum_<GJKVariant>("GJKVariant")
        .value("DefaultGJK", GJKVariant::DefaultGJK)
        .value("PolyakAcceleration", GJKVariant::PolyakAcceleration)
        .value("NesterovAcceleration", GJKVariant::NesterovAcceleration)
        .export_values();
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<GJKInitialGuess>()) {
    enum_<GJKInitialGuess>("GJKInitialGuess")
        .value("DefaultGuess", GJKInitialGuess::DefaultGuess)
        .value("CachedGuess", GJKInitialGuess::CachedGuess)
        .value("BoundingVolumeGuess", GJKInitialGuess::BoundingVolumeGuess)
        .export_values();
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          GJKConvergenceCriterion>()) {
    enum_<GJKConvergenceCriterion>("GJKConvergenceCriterion")
        .value("Default", GJKConvergenceCriterion::Default)
        .value("DualityGap", GJKConvergenceCriterion::DualityGap)
        .value("Hybrid", GJKConvergenceCriterion::Hybrid)
        .export_values();
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<
          GJKConvergenceCriterionType>()) {
    enum_<GJKConvergenceCriterionType>("GJKConvergenceCriterionType")
        .value("Absolute", GJKConvergenceCriterionType::Absolute)
        .value("Relative", GJKConvergenceCriterionType::Relative)
        .export_values();
  }

  if (!eigenpy::register_symbolic_link_to_registered_type<GJK>()) {
    class_<GJK>("GJK", doxygen::class_doc<GJK>(), no_init)
        .def(doxygen::visitor::init<GJK, unsigned int, FCL_REAL>())
        .DEF_RW_CLASS_ATTRIB(GJK, distance)
        .DEF_RW_CLASS_ATTRIB(GJK, ray)
        .DEF_RW_CLASS_ATTRIB(GJK, support_hint)
        .DEF_RW_CLASS_ATTRIB(GJK, gjk_variant)
        .DEF_RW_CLASS_ATTRIB(GJK, convergence_criterion)
        .DEF_RW_CLASS_ATTRIB(GJK, convergence_criterion_type)
        .DEF_CLASS_FUNC(GJK, reset)
        .DEF_CLASS_FUNC(GJK, evaluate)
        .DEF_CLASS_FUNC(GJK, getTolerance)
        .DEF_CLASS_FUNC(GJK, getNumMaxIterations)
        .DEF_CLASS_FUNC(GJK, getNumIterations)
        .DEF_CLASS_FUNC(GJK, getNumIterationsMomentumStopped)
        .DEF_CLASS_FUNC(GJK, hasClosestPoints)
        .DEF_CLASS_FUNC(GJK, getClosestPoints)
        .DEF_CLASS_FUNC(GJK, setDistanceEarlyBreak)
        .DEF_CLASS_FUNC(GJK, getGuessFromSimplex);
  }
}
