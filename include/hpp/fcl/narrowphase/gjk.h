/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

/** \author Jia Pan */

#ifndef HPP_FCL_GJK_H
#define HPP_FCL_GJK_H

#include <vector>

#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/math/transform.h>

namespace hpp {
namespace fcl {

namespace details {

/// @brief the support function for shape
/// \param hint use to initialize the search when shape is a ConvexBase object.
Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir, bool dirIsNormalized,
                 int& hint);

/// @brief Minkowski difference class of two shapes
///
/// @note The Minkowski difference is expressed in the frame of the first shape.
struct HPP_FCL_DLLAPI MinkowskiDiff {
  typedef Eigen::Array<FCL_REAL, 1, 2> Array2d;

  /// @brief points to two shapes
  const ShapeBase* shapes[2];

  struct ShapeData {
    std::vector<int8_t> visited;
  };

  /// @brief Store temporary data for the computation of the support point for
  /// each shape.
  ShapeData data[2];

  /// @brief rotation from shape1 to shape0
  /// such that @f$ p_in_0 = oR1 * p_in_1 + ot1 @f$.
  Matrix3f oR1;

  /// @brief translation from shape1 to shape0
  /// such that @f$ p_in_0 = oR1 * p_in_1 + ot1 @f$.
  Vec3f ot1;

  /// @brief The radius of the sphere swepted volume.
  /// The 2 values correspond to the inflation of shape 0 and shape 1/
  /// These inflation values are used for Sphere and Capsule.
  Array2d inflation;

  /// @brief Number of points in a Convex object from which using a logarithmic
  /// support function is faster than a linear one.
  /// It defaults to 32.
  /// \note It must set before the call to \ref set.
  int linear_log_convex_threshold;

  /// @brief Wether or not to use the normalize heuristic in the GJK Nesterov
  /// acceleration. This setting is only applied if the Nesterov acceleration in
  /// the GJK class is active.
  bool normalize_support_direction;

  typedef void (*GetSupportFunction)(const MinkowskiDiff& minkowskiDiff,
                                     const Vec3f& dir, bool dirIsNormalized,
                                     Vec3f& support0, Vec3f& support1,
                                     support_func_guess_t& hint,
                                     ShapeData data[2]);
  GetSupportFunction getSupportFunc;

  MinkowskiDiff()
      : linear_log_convex_threshold(32),
        normalize_support_direction(false),
        getSupportFunc(NULL) {}

  /// Set the two shapes,
  /// assuming the relative transformation between them is identity.
  void set(const ShapeBase* shape0, const ShapeBase* shape1);

  /// Set the two shapes, with a relative transformation.
  void set(const ShapeBase* shape0, const ShapeBase* shape1,
           const Transform3f& tf0, const Transform3f& tf1);

  /// @brief support function for shape0
  inline Vec3f support0(const Vec3f& d, bool dIsNormalized, int& hint) const {
    return getSupport(shapes[0], d, dIsNormalized, hint);
  }

  /// @brief support function for shape1
  inline Vec3f support1(const Vec3f& d, bool dIsNormalized, int& hint) const {
    return oR1 *
               getSupport(shapes[1], oR1.transpose() * d, dIsNormalized, hint) +
           ot1;
  }

  /// @brief support function for the pair of shapes
  inline void support(const Vec3f& d, bool dIsNormalized, Vec3f& supp0,
                      Vec3f& supp1, support_func_guess_t& hint) const {
    assert(getSupportFunc != NULL);
    getSupportFunc(*this, d, dIsNormalized, supp0, supp1, hint,
                   const_cast<ShapeData*>(data));
  }
};

/// @brief class for GJK algorithm
///
/// @note The computations are performed in the frame of the first shape.
struct HPP_FCL_DLLAPI GJK {
  struct HPP_FCL_DLLAPI SimplexV {
    /// @brief support vector for shape 0 and 1.
    Vec3f w0, w1;
    /// @brief support vector (i.e., the furthest point on the shape along the
    /// support direction)
    Vec3f w;
  };

  typedef unsigned char vertex_id_t;

  struct HPP_FCL_DLLAPI Simplex {
    /// @brief simplex vertex
    SimplexV* vertex[4];
    /// @brief size of simplex (number of vertices)
    vertex_id_t rank;

    Simplex() {}
  };

  /// @brief Status of the GJK algorithm:
  /// Valid: GJK converged and the shapes are not in collision.
  /// Inside: GJK converged and the shapes are in collision.
  /// Failed: GJK did not converge.
  enum Status { Valid, Inside, Failed, EarlyStopped };

  MinkowskiDiff const* shape;
  Vec3f ray;
  GJKVariant gjk_variant;
  GJKConvergenceCriterion convergence_criterion;
  GJKConvergenceCriterionType convergence_criterion_type;
  support_func_guess_t support_hint;
  /// The distance computed by GJK. The possible values are
  /// - \f$ d = - R - 1 \f$ when a collision is detected and GJK
  ///   cannot compute penetration informations.
  /// - \f$ - R \le d \le 0 \f$ when a collision is detected and GJK can
  ///   compute penetration informations.
  /// - \f$ 0 < d \le d_{ub} \f$ when there is no collision and GJK can compute
  ///   the closest points.
  /// - \f$ d_{ub} < d \f$ when there is no collision and GJK cannot compute the
  ///   closest points.
  ///
  /// where \f$ d \f$ is the GJK::distance, \f$ R \f$ is the sum of the \c shape
  /// MinkowskiDiff::inflation and \f$ d_{ub} \f$ is the
  /// GJK::distance_upper_bound.
  FCL_REAL distance;
  Simplex simplices[2];

  /// \param max_iterations_ number of iteration before GJK returns failure.
  /// \param tolerance_ precision of the algorithm.
  ///
  /// The tolerance argument is useful for continuous shapes and for polyhedron
  /// with some vertices closer than this threshold.
  ///
  /// Suggested values are 100 iterations and a tolerance of 1e-6.
  GJK(unsigned int max_iterations_, FCL_REAL tolerance_)
      : max_iterations(max_iterations_), tolerance(tolerance_) {
    initialize();
  }

  void initialize();

  /// @brief GJK algorithm, given the initial value guess
  Status evaluate(
      const MinkowskiDiff& shape, const Vec3f& guess,
      const support_func_guess_t& supportHint = support_func_guess_t::Zero());

  /// @brief apply the support function along a direction, the result is return
  /// in sv
  inline void getSupport(const Vec3f& d, bool dIsNormalized, SimplexV& sv,
                         support_func_guess_t& hint) const {
    shape->support(d, dIsNormalized, sv.w0, sv.w1, hint);
    sv.w = sv.w0 - sv.w1;
  }

  /// @brief whether the simplex enclose the origin
  bool encloseOrigin();

  /// @brief get the underlying simplex using in GJK, can be used for cache in
  /// next iteration
  inline Simplex* getSimplex() const { return simplex; }

  /// Tells whether the closest points are available.
  bool hasClosestPoints() { return distance < distance_upper_bound; }

  /// Tells whether the penetration information.
  ///
  /// In such case, most indepth points and penetration depth can be retrieved
  /// from GJK. Calling EPA has an undefined behaviour.
  bool hasPenetrationInformation(const MinkowskiDiff& shape) {
    return distance > -shape.inflation.sum();
  }

  /// Get the closest points on each object.
  /// @return true on success
  bool getClosestPoints(const MinkowskiDiff& shape, Vec3f& w0, Vec3f& w1);

  /// @brief get the guess from current simplex
  Vec3f getGuessFromSimplex() const;

  /// @brief Distance threshold for early break.
  /// GJK stops when it proved the distance is more than this threshold.
  /// @note The closest points will be erroneous in this case.
  ///       If you want the closest points, set this to infinity (the default).
  void setDistanceEarlyBreak(const FCL_REAL& dup) {
    distance_upper_bound = dup;
  }

  /// @brief Convergence check used to stop GJK when shapes are not in
  /// collision.
  bool checkConvergence(const Vec3f& w, const FCL_REAL& rl, FCL_REAL& alpha,
                        const FCL_REAL& omega);

  /// @brief Get GJK number of iterations.
  inline size_t getIterations() { return iterations; }

  /// @brief Get GJK tolerance.
  inline FCL_REAL getTolerance() { return tolerance; }

 private:
  SimplexV store_v[4];
  SimplexV* free_v[4];
  vertex_id_t nfree;
  vertex_id_t current;
  Simplex* simplex;
  Status status;

  unsigned int max_iterations;
  FCL_REAL tolerance;
  FCL_REAL distance_upper_bound;
  size_t iterations;

  /// @brief discard one vertex from the simplex
  inline void removeVertex(Simplex& simplex);

  /// @brief append one vertex to the simplex
  inline void appendVertex(Simplex& simplex, const Vec3f& v, bool isNormalized,
                           support_func_guess_t& hint);

  /// @brief Project origin (0) onto line a-b
  /// For a detailed explanation of how to efficiently project onto a simplex,
  /// check out Ericson's book, page 403:
  /// https://realtimecollisiondetection.net/ To sum up, a simplex has a voronoi
  /// region for each feature it has (vertex, edge, face). We find the voronoi
  /// region in which the origin lies and stop as soon as we find it; we then
  /// project onto it and return the result. We start by voronoi regions
  /// generated by vertices then move on to edges then faces. Checking voronoi
  /// regions is done using simple dot products. Moreover, edges voronoi checks
  /// reuse computations of vertices voronoi checks. The same goes for faces
  /// which reuse checks from edges.
  /// Finally, in addition to the voronoi procedure, checks relying on the order
  /// of construction
  // of the simplex are added. To know more about these, visit
  // https://caseymuratori.com/blog_0003.
  bool projectLineOrigin(const Simplex& current, Simplex& next);

  /// @brief Project origin (0) onto triangle a-b-c
  /// See \ref projectLineOrigin for an explanation on simplex projections.
  bool projectTriangleOrigin(const Simplex& current, Simplex& next);

  /// @brief Project origin (0) onto tetrahedron a-b-c-d
  /// See \ref projectLineOrigin for an explanation on simplex projections.
  bool projectTetrahedraOrigin(const Simplex& current, Simplex& next);
};

static const size_t EPA_MAX_FACES = 128;
static const size_t EPA_MAX_VERTICES = 64;
static const FCL_REAL EPA_EPS = 0.000001;
static const size_t EPA_MAX_ITERATIONS = 255;

/// @brief class for EPA algorithm
struct HPP_FCL_DLLAPI EPA {
  typedef GJK::SimplexV SimplexV;
  struct HPP_FCL_DLLAPI SimplexF {
    Vec3f n;
    FCL_REAL d;
    SimplexV* vertex[3];  // a face has three vertices
    SimplexF* f[3];       // a face has three adjacent faces
    SimplexF* l[2];       // the pre and post faces in the list
    size_t e[3];
    size_t pass;

    SimplexF() : n(Vec3f::Zero()){};
  };

  struct HPP_FCL_DLLAPI SimplexList {
    SimplexF* root;
    size_t count;
    SimplexList() : root(NULL), count(0) {}
    void append(SimplexF* face) {
      face->l[0] = NULL;
      face->l[1] = root;
      if (root) root->l[0] = face;
      root = face;
      ++count;
    }

    void remove(SimplexF* face) {
      if (face->l[1]) face->l[1]->l[0] = face->l[0];
      if (face->l[0]) face->l[0]->l[1] = face->l[1];
      if (face == root) root = face->l[1];
      --count;
    }
  };

  static inline void bind(SimplexF* fa, size_t ea, SimplexF* fb, size_t eb) {
    fa->e[ea] = eb;
    fa->f[ea] = fb;
    fb->e[eb] = ea;
    fb->f[eb] = fa;
  }

  struct HPP_FCL_DLLAPI SimplexHorizon {
    SimplexF* cf;  // current face in the horizon
    SimplexF* ff;  // first face in the horizon
    size_t nf;     // number of faces in the horizon
    SimplexHorizon() : cf(NULL), ff(NULL), nf(0) {}
  };

 private:
  unsigned int max_face_num;
  unsigned int max_vertex_num;
  unsigned int max_iterations;
  FCL_REAL tolerance;

 public:
  enum Status {
    Failed = 0,
    Valid = 1,
    AccuracyReached = 1 << 1 | Valid,
    Degenerated = 1 << 1 | Failed,
    NonConvex = 2 << 1 | Failed,
    InvalidHull = 3 << 1 | Failed,
    OutOfFaces = 4 << 1 | Failed,
    OutOfVertices = 5 << 1 | Failed,
    FallBack = 6 << 1 | Failed
  };

  Status status;
  GJK::Simplex result;
  Vec3f normal;
  FCL_REAL depth;
  SimplexV* sv_store;
  SimplexF* fc_store;
  size_t nextsv;
  SimplexList hull, stock;

  EPA(unsigned int max_face_num_, unsigned int max_vertex_num_,
      unsigned int max_iterations_, FCL_REAL tolerance_)
      : max_face_num(max_face_num_),
        max_vertex_num(max_vertex_num_),
        max_iterations(max_iterations_),
        tolerance(tolerance_) {
    initialize();
  }

  ~EPA() {
    delete[] sv_store;
    delete[] fc_store;
  }

  void initialize();

  /// \return a Status which can be demangled using (status & Valid) or
  ///         (status & Failed). The other values provide a more detailled
  ///         status
  Status evaluate(GJK& gjk, const Vec3f& guess);

  /// Get the closest points on each object.
  /// @return true on success
  bool getClosestPoints(const MinkowskiDiff& shape, Vec3f& w0, Vec3f& w1);

 private:
  bool getEdgeDist(SimplexF* face, SimplexV* a, SimplexV* b, FCL_REAL& dist);

  SimplexF* newFace(SimplexV* a, SimplexV* b, SimplexV* vertex, bool forced);

  /// @brief Find the best polytope face to split
  SimplexF* findBest();

  /// @brief the goal is to add a face connecting vertex w and face edge f[e]
  bool expand(size_t pass, SimplexV* w, SimplexF* f, size_t e,
              SimplexHorizon& horizon);
};

}  // namespace details

}  // namespace fcl

}  // namespace hpp

#endif
