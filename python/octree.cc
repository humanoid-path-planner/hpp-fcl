
#include "fcl.hh"

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/octree.h>

#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#endif

void exposeOctree() {
  using namespace hpp::fcl;
  namespace bp = boost::python;
  namespace dv = doxygen::visitor;

  bp::class_<OcTree, bp::bases<CollisionGeometry>, shared_ptr<OcTree> >(
      "OcTree", doxygen::class_doc<OcTree>(), bp::no_init)
      .def(dv::init<OcTree, FCL_REAL>())
      .def(dv::member_func("getTreeDepth", &OcTree::getTreeDepth))
      .def(dv::member_func("getOccupancyThres", &OcTree::getOccupancyThres))
      .def(dv::member_func("getFreeThres", &OcTree::getFreeThres))
      .def(dv::member_func("getDefaultOccupancy", &OcTree::getDefaultOccupancy))
      .def(dv::member_func("setCellDefaultOccupancy",
                           &OcTree::setCellDefaultOccupancy))
      .def(dv::member_func("setOccupancyThres", &OcTree::setOccupancyThres))
      .def(dv::member_func("setFreeThres", &OcTree::setFreeThres))
      .def(dv::member_func("getRootBV", &OcTree::getRootBV));

  doxygen::def("makeOctree", &makeOctree);
}
