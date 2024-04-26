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

#include "hpp/fcl/contact_patch_func_matrix.h"
#include "hpp/fcl/shape/geometric_shapes.h"
#include "hpp/fcl/internal/shape_shape_contact_patch_func.h"

namespace hpp {
namespace fcl {

ContactPatchFunctionMatrix::ContactPatchFunctionMatrix() {
  for (int i = 0; i < NODE_COUNT; ++i) {
    for (int j = 0; j < NODE_COUNT; ++j) contact_patch_matrix[i][j] = nullptr;
  }

  // clang-format off
  contact_patch_matrix[GEOM_BOX][GEOM_BOX]             = &ShapeShapeContactPatch<Box, Box>;
  contact_patch_matrix[GEOM_BOX][GEOM_SPHERE]          = &ShapeShapeContactPatch<Box, Sphere>;
  contact_patch_matrix[GEOM_BOX][GEOM_CAPSULE]         = &ShapeShapeContactPatch<Box, Capsule>;
  contact_patch_matrix[GEOM_BOX][GEOM_CONE]            = &ShapeShapeContactPatch<Box, Cone>;
  contact_patch_matrix[GEOM_BOX][GEOM_CYLINDER]        = &ShapeShapeContactPatch<Box, Cylinder>;
  contact_patch_matrix[GEOM_BOX][GEOM_CONVEX]          = &ShapeShapeContactPatch<Box, ConvexBase>;
  contact_patch_matrix[GEOM_BOX][GEOM_PLANE]           = &ShapeShapeContactPatch<Box, Plane>;
  contact_patch_matrix[GEOM_BOX][GEOM_HALFSPACE]       = &ShapeShapeContactPatch<Box, Halfspace>;
  contact_patch_matrix[GEOM_BOX][GEOM_ELLIPSOID]       = &ShapeShapeContactPatch<Box, Ellipsoid>;
  contact_patch_matrix[GEOM_BOX][GEOM_TRIANGLE]        = &ShapeShapeContactPatch<Box, TriangleP>;

  contact_patch_matrix[GEOM_SPHERE][GEOM_BOX]          = &ShapeShapeContactPatch<Sphere, Box>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_SPHERE]       = &ShapeShapeContactPatch<Sphere, Sphere>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_CAPSULE]      = &ShapeShapeContactPatch<Sphere, Capsule>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_CONE]         = &ShapeShapeContactPatch<Sphere, Cone>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_CYLINDER]     = &ShapeShapeContactPatch<Sphere, Cylinder>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_CONVEX]       = &ShapeShapeContactPatch<Sphere, ConvexBase>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_PLANE]        = &ShapeShapeContactPatch<Sphere, Plane>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_HALFSPACE]    = &ShapeShapeContactPatch<Sphere, Halfspace>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_ELLIPSOID]    = &ShapeShapeContactPatch<Sphere, Ellipsoid>;
  contact_patch_matrix[GEOM_SPHERE][GEOM_TRIANGLE]     = &ShapeShapeContactPatch<Sphere, TriangleP>;

  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_BOX]       = &ShapeShapeContactPatch<Ellipsoid, Box>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_SPHERE]    = &ShapeShapeContactPatch<Ellipsoid, Sphere>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_CAPSULE]   = &ShapeShapeContactPatch<Ellipsoid, Capsule>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_CONE]      = &ShapeShapeContactPatch<Ellipsoid, Cone>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_CYLINDER]  = &ShapeShapeContactPatch<Ellipsoid, Cylinder>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_CONVEX]    = &ShapeShapeContactPatch<Ellipsoid, ConvexBase>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_PLANE]     = &ShapeShapeContactPatch<Ellipsoid, Plane>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_HALFSPACE] = &ShapeShapeContactPatch<Ellipsoid, Halfspace>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_ELLIPSOID] = &ShapeShapeContactPatch<Ellipsoid, Ellipsoid>;
  contact_patch_matrix[GEOM_ELLIPSOID][GEOM_TRIANGLE]  = &ShapeShapeContactPatch<Ellipsoid, TriangleP>;

  contact_patch_matrix[GEOM_CAPSULE][GEOM_BOX]         = &ShapeShapeContactPatch<Capsule, Box>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_SPHERE]      = &ShapeShapeContactPatch<Capsule, Sphere>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_CAPSULE]     = &ShapeShapeContactPatch<Capsule, Capsule>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_CONE]        = &ShapeShapeContactPatch<Capsule, Cone>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_CYLINDER]    = &ShapeShapeContactPatch<Capsule, Cylinder>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_CONVEX]      = &ShapeShapeContactPatch<Capsule, ConvexBase>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_PLANE]       = &ShapeShapeContactPatch<Capsule, Plane>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_HALFSPACE]   = &ShapeShapeContactPatch<Capsule, Halfspace>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_ELLIPSOID]   = &ShapeShapeContactPatch<Capsule, Ellipsoid>;
  contact_patch_matrix[GEOM_CAPSULE][GEOM_TRIANGLE]    = &ShapeShapeContactPatch<Capsule, TriangleP>;

  contact_patch_matrix[GEOM_CONE][GEOM_BOX]            = &ShapeShapeContactPatch<Cone, Box>;
  contact_patch_matrix[GEOM_CONE][GEOM_SPHERE]         = &ShapeShapeContactPatch<Cone, Sphere>;
  contact_patch_matrix[GEOM_CONE][GEOM_CAPSULE]        = &ShapeShapeContactPatch<Cone, Capsule>;
  contact_patch_matrix[GEOM_CONE][GEOM_CONE]           = &ShapeShapeContactPatch<Cone, Cone>;
  contact_patch_matrix[GEOM_CONE][GEOM_CYLINDER]       = &ShapeShapeContactPatch<Cone, Cylinder>;
  contact_patch_matrix[GEOM_CONE][GEOM_CONVEX]         = &ShapeShapeContactPatch<Cone, ConvexBase>;
  contact_patch_matrix[GEOM_CONE][GEOM_PLANE]          = &ShapeShapeContactPatch<Cone, Plane>;
  contact_patch_matrix[GEOM_CONE][GEOM_HALFSPACE]      = &ShapeShapeContactPatch<Cone, Halfspace>;
  contact_patch_matrix[GEOM_CONE][GEOM_ELLIPSOID]      = &ShapeShapeContactPatch<Cone, Ellipsoid>;
  contact_patch_matrix[GEOM_CONE][GEOM_TRIANGLE]       = &ShapeShapeContactPatch<Cone, TriangleP>;

  contact_patch_matrix[GEOM_CYLINDER][GEOM_BOX]        = &ShapeShapeContactPatch<Cylinder, Box>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_SPHERE]     = &ShapeShapeContactPatch<Cylinder, Sphere>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_CAPSULE]    = &ShapeShapeContactPatch<Cylinder, Capsule>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_CONE]       = &ShapeShapeContactPatch<Cylinder, Cone>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_CYLINDER]   = &ShapeShapeContactPatch<Cylinder, Cylinder>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_CONVEX]     = &ShapeShapeContactPatch<Cylinder, ConvexBase>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_PLANE]      = &ShapeShapeContactPatch<Cylinder, Plane>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_HALFSPACE]  = &ShapeShapeContactPatch<Cylinder, Halfspace>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_ELLIPSOID]  = &ShapeShapeContactPatch<Cylinder, Ellipsoid>;
  contact_patch_matrix[GEOM_CYLINDER][GEOM_TRIANGLE]   = &ShapeShapeContactPatch<Cylinder, TriangleP>;

  contact_patch_matrix[GEOM_CONVEX][GEOM_BOX]          = &ShapeShapeContactPatch<ConvexBase, Box>;
  contact_patch_matrix[GEOM_CONVEX][GEOM_SPHERE]       = &ShapeShapeContactPatch<ConvexBase, Sphere>;
  contact_patch_matrix[GEOM_CONVEX][GEOM_CAPSULE]      = &ShapeShapeContactPatch<ConvexBase, Capsule>;
  contact_patch_matrix[GEOM_CONVEX][GEOM_CONE]         = &ShapeShapeContactPatch<ConvexBase, Cone>;
  contact_patch_matrix[GEOM_CONVEX][GEOM_CYLINDER]     = &ShapeShapeContactPatch<ConvexBase, Cylinder>;
  contact_patch_matrix[GEOM_CONVEX][GEOM_CONVEX]       = &ShapeShapeContactPatch<ConvexBase, ConvexBase>;
  contact_patch_matrix[GEOM_CONVEX][GEOM_PLANE]        = &ShapeShapeContactPatch<ConvexBase, Plane>;
  contact_patch_matrix[GEOM_CONVEX][GEOM_HALFSPACE]    = &ShapeShapeContactPatch<ConvexBase, Halfspace>;
  contact_patch_matrix[GEOM_CONVEX][GEOM_ELLIPSOID]    = &ShapeShapeContactPatch<ConvexBase, Ellipsoid>;
  contact_patch_matrix[GEOM_CONVEX][GEOM_TRIANGLE]     = &ShapeShapeContactPatch<ConvexBase, TriangleP>;

  contact_patch_matrix[GEOM_PLANE][GEOM_BOX]           = &ShapeShapeContactPatch<Plane, Box>;
  contact_patch_matrix[GEOM_PLANE][GEOM_SPHERE]        = &ShapeShapeContactPatch<Plane, Sphere>;
  contact_patch_matrix[GEOM_PLANE][GEOM_CAPSULE]       = &ShapeShapeContactPatch<Plane, Capsule>;
  contact_patch_matrix[GEOM_PLANE][GEOM_CONE]          = &ShapeShapeContactPatch<Plane, Cone>;
  contact_patch_matrix[GEOM_PLANE][GEOM_CYLINDER]      = &ShapeShapeContactPatch<Plane, Cylinder>;
  contact_patch_matrix[GEOM_PLANE][GEOM_CONVEX]        = &ShapeShapeContactPatch<Plane, ConvexBase>;
  contact_patch_matrix[GEOM_PLANE][GEOM_PLANE]         = &ShapeShapeContactPatch<Plane, Plane>;
  contact_patch_matrix[GEOM_PLANE][GEOM_HALFSPACE]     = &ShapeShapeContactPatch<Plane, Halfspace>;
  contact_patch_matrix[GEOM_PLANE][GEOM_ELLIPSOID]     = &ShapeShapeContactPatch<Plane, Ellipsoid>;
  contact_patch_matrix[GEOM_PLANE][GEOM_TRIANGLE]      = &ShapeShapeContactPatch<Plane, TriangleP>;

  contact_patch_matrix[GEOM_HALFSPACE][GEOM_BOX]       = &ShapeShapeContactPatch<Halfspace, Box>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_SPHERE]    = &ShapeShapeContactPatch<Halfspace, Sphere>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_CAPSULE]   = &ShapeShapeContactPatch<Halfspace, Capsule>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_CONE]      = &ShapeShapeContactPatch<Halfspace, Cone>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_CYLINDER]  = &ShapeShapeContactPatch<Halfspace, Cylinder>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_CONVEX]    = &ShapeShapeContactPatch<Halfspace, ConvexBase>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_PLANE]     = &ShapeShapeContactPatch<Halfspace, Plane>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeContactPatch<Halfspace, Halfspace>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_ELLIPSOID] = &ShapeShapeContactPatch<Halfspace, Ellipsoid>;
  contact_patch_matrix[GEOM_HALFSPACE][GEOM_TRIANGLE]  = &ShapeShapeContactPatch<Halfspace, TriangleP>;

  contact_patch_matrix[GEOM_TRIANGLE][GEOM_BOX]        = &ShapeShapeContactPatch<TriangleP, Box>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_SPHERE]     = &ShapeShapeContactPatch<TriangleP, Sphere>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_CAPSULE]    = &ShapeShapeContactPatch<TriangleP, Capsule>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_CONE]       = &ShapeShapeContactPatch<TriangleP, Cone>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_CYLINDER]   = &ShapeShapeContactPatch<TriangleP, Cylinder>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_CONVEX]     = &ShapeShapeContactPatch<TriangleP, ConvexBase>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_PLANE]      = &ShapeShapeContactPatch<TriangleP, Plane>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_HALFSPACE]  = &ShapeShapeContactPatch<TriangleP, Halfspace>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_ELLIPSOID]  = &ShapeShapeContactPatch<TriangleP, Ellipsoid>;
  contact_patch_matrix[GEOM_TRIANGLE][GEOM_TRIANGLE]   = &ShapeShapeContactPatch<TriangleP, TriangleP>;
  // clang-format on
}

}  // namespace fcl
}  // namespace hpp
