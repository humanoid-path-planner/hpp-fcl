#include "hpp/fcl/narrowphase/minkowski_difference.h"

namespace hpp {
namespace fcl {
namespace details {

// Explicit instantiation
// clang-format off
template void HPP_FCL_DLLAPI MinkowskiDiff::set<SupportOptions::NoSweptSphere>(const ShapeBase*, const ShapeBase*);
template void HPP_FCL_DLLAPI MinkowskiDiff::set<SupportOptions::WithSweptSphere>(const ShapeBase*, const ShapeBase*);

template void HPP_FCL_DLLAPI MinkowskiDiff::set<SupportOptions::NoSweptSphere>(const ShapeBase*, const ShapeBase*, const Transform3f&, const Transform3f&);
template void HPP_FCL_DLLAPI MinkowskiDiff::set<SupportOptions::WithSweptSphere>(const ShapeBase*, const ShapeBase*, const Transform3f&, const Transform3f&);

template Vec3f HPP_FCL_DLLAPI MinkowskiDiff::support0<SupportOptions::NoSweptSphere>(const Vec3f&, int&) const;
template Vec3f HPP_FCL_DLLAPI MinkowskiDiff::support0<SupportOptions::WithSweptSphere>(const Vec3f&, int&) const;

template Vec3f HPP_FCL_DLLAPI MinkowskiDiff::support1<SupportOptions::NoSweptSphere>(const Vec3f&, int&) const;
template Vec3f HPP_FCL_DLLAPI MinkowskiDiff::support1<SupportOptions::WithSweptSphere>(const Vec3f&, int&) const;
// clang-format on

}  // namespace details
}  // namespace fcl
}  // namespace hpp
