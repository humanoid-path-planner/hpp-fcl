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

#ifndef HPP_FCL_COLLISION_UTILITY_H
#define HPP_FCL_COLLISION_UTILITY_H

#include <hpp/fcl/collision_object.h>

namespace hpp
{
namespace fcl
{
  HPP_FCL_DLLAPI CollisionGeometry* extract(const CollisionGeometry* model,
                                            const Transform3f& pose,
                                            const AABB& aabb);
}

} // namespace hpp

#endif // FCL_COLLISION_UTILITY_H
