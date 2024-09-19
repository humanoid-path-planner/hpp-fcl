/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, INRIA
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

/** \author Justin Carpentier */

#include "coal/hfield.h"

#include "coal/BV/BV.h"
#include "coal/shape/convex.h"

#include "coal/internal/BV_splitter.h"
#include "coal/internal/BV_fitter.h"

#include <iostream>
#include <string.h>

namespace coal {

template <>
NODE_TYPE HeightField<AABB>::getNodeType() const {
  return HF_AABB;
}

template <>
NODE_TYPE HeightField<OBB>::getNodeType() const {
  return BV_UNKNOWN;  // HF_OBB;
}

template <>
NODE_TYPE HeightField<RSS>::getNodeType() const {
  return BV_UNKNOWN;  // HF_RSS;
}

template <>
NODE_TYPE HeightField<kIOS>::getNodeType() const {
  return BV_UNKNOWN;  // BV_kIOS;
}

template <>
NODE_TYPE HeightField<OBBRSS>::getNodeType() const {
  return HF_OBBRSS;
}

template <>
NODE_TYPE HeightField<KDOP<16> >::getNodeType() const {
  return BV_UNKNOWN;  // BV_KDOP16;
}

template <>
NODE_TYPE HeightField<KDOP<18> >::getNodeType() const {
  return BV_UNKNOWN;  // BV_KDOP18;
}

template <>
NODE_TYPE HeightField<KDOP<24> >::getNodeType() const {
  return BV_UNKNOWN;  // BV_KDOP24;
}

// template class HeightField<KDOP<16> >;
// template class HeightField<KDOP<18> >;
// template class HeightField<KDOP<24> >;
template class HeightField<OBB>;
template class HeightField<AABB>;
template class HeightField<RSS>;
// template class HeightField<kIOS>;
template class HeightField<OBBRSS>;

}  // namespace coal
