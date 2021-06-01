#include <hpp/fcl/fwd.hh>
#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
namespace doxygen {
  using hpp::fcl::shared_ptr;
}
#endif

// Silence a warning about a deprecated use of boost bind by boost python
// at least fo boost 1.73 to 1.75
// ref. https://github.com/stack-of-tasks/tsid/issues/128
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#undef BOOST_BIND_GLOBAL_PLACEHOLDERS

void exposeVersion();

void exposeMaths();

void exposeCollisionGeometries();

void exposeCollisionObject();

void exposeMeshLoader();

void exposeCollisionAPI();

void exposeDistanceAPI();

void exposeGJK();

#ifdef HPP_FCL_HAS_OCTOMAP
void exposeOctree();
#endif
