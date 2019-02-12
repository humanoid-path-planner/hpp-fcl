// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-fcl.
// hpp-fcl is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-fcl is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-fcl. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/fcl/collision_utility.h>

#include <hpp/fcl/BVH/BVH_utility.h>

namespace hpp
{
namespace fcl
{
  namespace details {
    template<typename NT>
    inline CollisionGeometry* extractBVHtpl (const CollisionGeometry* model, const Transform3f& pose, const AABB& aabb)
    {
      const BVHModel<NT>* m = static_cast<const BVHModel<NT>*>(model);
      return BVHExtract(*m, pose, aabb);
    }
    CollisionGeometry* extractBVH (const CollisionGeometry* model, const Transform3f& pose, const AABB& aabb)
    {
      switch (model->getNodeType()) {
        case BV_AABB  : return extractBVHtpl<AABB     >(model, pose, aabb);
        case BV_OBB   : return extractBVHtpl<OBB      >(model, pose, aabb);
        case BV_RSS   : return extractBVHtpl<RSS      >(model, pose, aabb);
        case BV_kIOS  : return extractBVHtpl<kIOS     >(model, pose, aabb);
        case BV_OBBRSS: return extractBVHtpl<OBBRSS   >(model, pose, aabb);
        case BV_KDOP16: return extractBVHtpl<KDOP<16> >(model, pose, aabb);
        case BV_KDOP18: return extractBVHtpl<KDOP<18> >(model, pose, aabb);
        case BV_KDOP24: return extractBVHtpl<KDOP<24> >(model, pose, aabb);
        default: throw std::runtime_error("Unknown type of bounding volume");
      }
    }
  }

  CollisionGeometry* extract(const CollisionGeometry* model, const Transform3f& pose, const AABB& aabb)
  {
    switch (model->getObjectType()) {
      case OT_BVH: return details::extractBVH (model, pose, aabb);
      // case OT_GEOM: return model;
      default: throw std::runtime_error("Extraction is not implemented for this type of object");
    }
  }
}

} // namespace hpp
