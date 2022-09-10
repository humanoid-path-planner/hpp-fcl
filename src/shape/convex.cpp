#include <hpp/fcl/shape/convex.h>

#ifdef HPP_FCL_HAS_QHULL
#include <libqhullcpp/QhullError.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullLinkedList.h>
#include <libqhullcpp/QhullVertex.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/QhullRidge.h>
#include <libqhullcpp/Qhull.h>

using orgQhull::Qhull;
using orgQhull::QhullFacet;
using orgQhull::QhullPoint;
using orgQhull::QhullRidgeSet;
using orgQhull::QhullVertex;
using orgQhull::QhullVertexList;
using orgQhull::QhullVertexSet;
#endif

namespace hpp {
namespace fcl {

// Reorders `tri` such that the dot product between the normal of triangle and
// the vector `triangle barycentre - convex_tri.center` is positive.
void reorderTriangle(const Convex<Triangle>* convex_tri, Triangle& tri) {
  Vec3f p0, p1, p2;
  p0 = convex_tri->points[tri[0]];
  p1 = convex_tri->points[tri[1]];
  p2 = convex_tri->points[tri[2]];

  Vec3f barycentre_tri, center_barycenter;
  barycentre_tri = (p0 + p1 + p2) / 3;
  center_barycenter = barycentre_tri - convex_tri->center;

  Vec3f edge_tri1, edge_tri2, n_tri;
  edge_tri1 = p1 - p0;
  edge_tri2 = p2 - p1;
  n_tri = edge_tri1.cross(edge_tri2);

  if (center_barycenter.dot(n_tri) < 0) {
    tri.set(tri[1], tri[0], tri[2]);
  }
}

ConvexBase* ConvexBase::convexHull(const Vec3f* pts, unsigned int num_points,
                                   bool keepTriangles,
                                   const char* qhullCommand) {
#ifdef HPP_FCL_HAS_QHULL
  if (num_points <= 3) {
    throw std::invalid_argument(
        "You shouldn't use this function with less than"
        " 4 points.");
  }
  assert(pts[0].data() + 3 == pts[1].data());

  Qhull qh;
  const char* command =
      qhullCommand ? qhullCommand : (keepTriangles ? "Qt" : "");
  qh.runQhull("", 3, static_cast<int>(num_points), pts[0].data(), command);

  if (qh.qhullStatus() != qh_ERRnone) {
    if (qh.hasQhullMessage()) std::cerr << qh.qhullMessage() << std::endl;
    throw std::logic_error("Qhull failed");
  }

  typedef std::size_t index_type;
  typedef int size_type;

  // Map index in pts to index in vertices. -1 means not used
  std::vector<int> pts_to_vertices(num_points, -1);

  // Initialize the vertices
  int nvertex = (qh.vertexCount());
  Vec3f* vertices = new Vec3f[size_t(nvertex)];
  QhullVertexList vertexList(qh.vertexList());
  int i_vertex = 0;
  for (QhullVertexList::const_iterator v = vertexList.begin();
       v != vertexList.end(); ++v) {
    QhullPoint pt((*v).point());
    pts_to_vertices[(size_t)pt.id()] = (int)i_vertex;
    vertices[i_vertex] = Vec3f(pt[0], pt[1], pt[2]);
    ++i_vertex;
  }
  assert(i_vertex == nvertex);

  Convex<Triangle>* convex_tri(NULL);
  ConvexBase* convex(NULL);
  if (keepTriangles)
    convex = convex_tri = new Convex<Triangle>();
  else
    convex = new ConvexBase;
  convex->initialize(true, vertices, static_cast<unsigned int>(nvertex));

  // Build the neighbors
  convex->neighbors = new Neighbors[size_t(nvertex)];
  std::vector<std::set<index_type> > nneighbors(static_cast<size_t>(nvertex));
  if (keepTriangles) {
    convex_tri->num_polygons = static_cast<unsigned int>(qh.facetCount());
    convex_tri->polygons = new Triangle[convex_tri->num_polygons];
    convex_tri->computeCenter();
  }

  unsigned int c_nneighbors = 0;
  unsigned int i_polygon = 0;

  // Compute the neighbors from the edges of the faces.
  for (QhullFacet facet = qh.beginFacet(); facet != qh.endFacet();
       facet = facet.next()) {
    if (facet.isSimplicial()) {
      // In 3D, simplicial faces have 3 vertices. We mark them as neighbors.
      QhullVertexSet f_vertices(facet.vertices());
      size_t n = static_cast<size_t>(f_vertices.count());
      assert(n == 3);
      Triangle tri(
          static_cast<size_t>(
              pts_to_vertices[static_cast<size_t>(f_vertices[0].point().id())]),
          static_cast<size_t>(
              pts_to_vertices[static_cast<size_t>(f_vertices[1].point().id())]),
          static_cast<size_t>(pts_to_vertices[static_cast<size_t>(
              f_vertices[2].point().id())]));
      if (keepTriangles) {
        reorderTriangle(convex_tri, tri);
        convex_tri->polygons[i_polygon++] = tri;
      }
      for (size_t j = 0; j < n; ++j) {
        size_t i = (j == 0) ? n - 1 : j - 1;
        size_t k = (j == n - 1) ? 0 : j + 1;
        // Update neighbors of pj;
        if (nneighbors[tri[j]].insert(tri[i]).second) c_nneighbors++;
        if (nneighbors[tri[j]].insert(tri[k]).second) c_nneighbors++;
      }
    } else {
      if (keepTriangles) {  // TODO I think there is a memory leak here.
        throw std::invalid_argument(
            "You requested to keep triangles so you "
            "must pass option \"Qt\" to qhull via the qhull command argument.");
      }
      // Non-simplicial faces have more than 3 vertices and contains a list of
      // rigdes. Ridges are (3-1)D simplex (i.e. one edge). We mark the two
      // vertices of each ridge as neighbors.
      QhullRidgeSet f_ridges(facet.ridges());
      for (size_type j = 0; j < f_ridges.count(); ++j) {
        assert(f_ridges[j].vertices().count() == 2);
        int pi = pts_to_vertices[static_cast<size_t>(
                f_ridges[j].vertices()[0].point().id())],
            pj = pts_to_vertices[static_cast<size_t>(
                f_ridges[j].vertices()[1].point().id())];
        // Update neighbors of pi and pj;
        if (nneighbors[static_cast<size_t>(pj)]
                .insert(static_cast<size_t>(pi))
                .second)
          c_nneighbors++;
        if (nneighbors[static_cast<size_t>(pi)]
                .insert(static_cast<size_t>(pj))
                .second)
          c_nneighbors++;
      }
    }
  }
  assert(!keepTriangles || i_polygon == qh.facetCount());

  // Fill the neighbor attribute of the returned object.
  convex->nneighbors_ = new unsigned int[c_nneighbors];
  unsigned int* p_nneighbors = convex->nneighbors_;
  for (size_t i = 0; i < static_cast<size_t>(nvertex); ++i) {
    Neighbors& n = convex->neighbors[i];
    if (nneighbors[i].size() >= (std::numeric_limits<unsigned char>::max)())
      throw std::logic_error("Too many neighbors.");
    n.count_ = (unsigned char)nneighbors[i].size();
    n.n_ = p_nneighbors;
    p_nneighbors =
        std::copy(nneighbors[i].begin(), nneighbors[i].end(), p_nneighbors);
  }
  assert(p_nneighbors == convex->nneighbors_ + c_nneighbors);
  return convex;
#else
  throw std::logic_error(
      "Library built without qhull. Cannot build object of this type.");
  HPP_FCL_UNUSED_VARIABLE(pts);
  HPP_FCL_UNUSED_VARIABLE(num_points);
  HPP_FCL_UNUSED_VARIABLE(keepTriangles);
  HPP_FCL_UNUSED_VARIABLE(qhullCommand);
#endif
}
}  // namespace fcl
}  // namespace hpp
