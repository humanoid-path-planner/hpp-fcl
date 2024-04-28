#include "hpp/fcl/narrowphase/support_functions.h"

namespace hpp {
namespace fcl {
namespace details {

// Explicit instantiation
// clang-format off
template Vec3f HPP_FCL_DLLAPI getSupport<SupportOptions::NoSweptSphere>(const ShapeBase*, const Vec3f&, int&);
template Vec3f HPP_FCL_DLLAPI getSupport<SupportOptions::WithSweptSphere>(const ShapeBase*, const Vec3f&, int&);
// clang-format on

}  // namespace details
}  // namespace fcl
}  // namespace hpp
