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

#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/math/transform.h>

namespace hpp
{
namespace fcl
{

namespace details
{

/// @brief the support function for shape
Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir, bool dirIsNormalized); 

/// @brief Minkowski difference class of two shapes
///
/// \todo template this by the two shapes. The triangle / triangle case can be
///       easily optimized computing once the triangle shapes[1] into frame0
///
/// \note The Minkowski difference is expressed in the frame of the first shape.
struct MinkowskiDiff
{
  /// @brief points to two shapes
  const ShapeBase* shapes[2];

  /// @brief rotation from shape1 to shape0
  /// such that \f$ p_in_0 = oR1 * p_in_1 + ot1 \f$.
  Matrix3f oR1;

  /// @brief translation from shape1 to shape0
  /// such that \f$ p_in_0 = oR1 * p_in_1 + ot1 \f$.
  Vec3f ot1;

  typedef void (*GetSupportFunction) (const MinkowskiDiff& minkowskiDiff,
      const Vec3f& dir, bool dirIsNormalized, Vec3f& support0, Vec3f& support1);
  GetSupportFunction getSupportFunc;

  MinkowskiDiff() : getSupportFunc (NULL) {}

  void set (const ShapeBase* shape0, const ShapeBase* shape1);

  void set (const Transform3f& tf0, const Transform3f& tf1)
  {
    oR1 = tf0.getRotation().transpose() * tf1.getRotation();
    ot1 = tf0.getRotation().transpose() * (tf1.getTranslation() - tf0.getTranslation());
  }

  /// @brief support function for shape0
  inline Vec3f support0(const Vec3f& d, bool dIsNormalized) const
  {
    return getSupport(shapes[0], d, dIsNormalized);
  }

  /// @brief support function for shape1
  inline Vec3f support1(const Vec3f& d, bool dIsNormalized) const
  {
    return oR1 * getSupport(shapes[1], oR1.transpose() * d, dIsNormalized) + ot1;
  }

  /// @brief support function for the pair of shapes
  inline void support(const Vec3f& d, bool dIsNormalized, Vec3f& supp0, Vec3f& supp1) const
  {
    assert(getSupportFunc != NULL);
    getSupportFunc(*this, d, dIsNormalized, supp0, supp1);
  }
};

/// @brief class for GJK algorithm
///
/// \note The computations are performed in the frame of the first shape.
struct GJK
{
  struct SimplexV
  {
    /// @brief support vector for shape 0 and 1.
    Vec3f w0, w1; 
    /// @brieg support vector (i.e., the furthest point on the shape along the support direction)
    Vec3f w;
  };

  struct Simplex
  {
    /// @brief simplex vertex
    SimplexV* vertex[4];
    /// @brief weight 
    FCL_REAL coefficient[4];
    /// @brief size of simplex (number of vertices)
    short rank;

    Simplex() : rank(0) {}
  };

  enum Status {Valid, Inside, Failed};

  MinkowskiDiff shape;
  Vec3f ray;
  FCL_REAL distance;
  Simplex simplices[2];


  GJK(unsigned int max_iterations_, FCL_REAL tolerance_)  : max_iterations(max_iterations_),
                                                            tolerance(tolerance_)
  {
    initialize(); 
  }
  
  void initialize();

  /// @brief GJK algorithm, given the initial value guess
  Status evaluate(const MinkowskiDiff& shape_, const Vec3f& guess);

  /// @brief apply the support function along a direction, the result is return in sv
  void getSupport(const Vec3f& d, bool dIsNormalized, SimplexV& sv) const;

  /// @brief discard one vertex from the simplex
  void removeVertex(Simplex& simplex);

  /// @brief append one vertex to the simplex
  void appendVertex(Simplex& simplex, const Vec3f& v, bool isNormalized = false);

  /// @brief whether the simplex enclose the origin
  bool encloseOrigin();

  /// @brief get the underlying simplex using in GJK, can be used for cache in next iteration
  inline Simplex* getSimplex() const
  {
    return simplex;
  }

  /// Get the closest points on each object.
  /// \return true on success
  static bool getClosestPoints (const Simplex& simplex, Vec3f& w0, Vec3f& w1);

  /// @brief get the guess from current simplex
  Vec3f getGuessFromSimplex() const;

private:
  SimplexV store_v[4];
  SimplexV* free_v[4];
  size_t nfree;
  size_t current;
  Simplex* simplex;
  Status status;

  unsigned int max_iterations;
  FCL_REAL tolerance;

  /// @brief Project origin (0) onto line a-b
  FCL_REAL projectLineOrigin(const Simplex& current, Simplex& next);

  /// @brief Project origin (0) onto triangle a-b-c
  FCL_REAL projectTriangleOrigin(const Simplex& current, Simplex& next);

};


static const size_t EPA_MAX_FACES = 128;
static const size_t EPA_MAX_VERTICES = 64;
static const FCL_REAL EPA_EPS = 0.000001;
static const size_t EPA_MAX_ITERATIONS = 255;

/// @brief class for EPA algorithm
struct EPA
{
private:
  typedef GJK::SimplexV SimplexV;
  struct SimplexF
  {
    Vec3f n;
    FCL_REAL d;
    SimplexV* vertex[3]; // a face has three vertices
    SimplexF* f[3]; // a face has three adjacent faces
    SimplexF* l[2]; // the pre and post faces in the list
    size_t e[3];
    size_t pass;

    SimplexF () : n(Vec3f::Zero()) {};
  };

  struct SimplexList
  {
    SimplexF* root;
    size_t count;
    SimplexList() : root(NULL), count(0) {}
    void append(SimplexF* face)
    {
      face->l[0] = NULL;
      face->l[1] = root;
      if(root) root->l[0] = face;
      root = face;
      ++count;
    }

    void remove(SimplexF* face)
    {
      if(face->l[1]) face->l[1]->l[0] = face->l[0];
      if(face->l[0]) face->l[0]->l[1] = face->l[1];
      if(face == root) root = face->l[1];
      --count;
    }
  };

  static inline void bind(SimplexF* fa, size_t ea, SimplexF* fb, size_t eb)
  {
    fa->e[ea] = eb; fa->f[ea] = fb;
    fb->e[eb] = ea; fb->f[eb] = fa;
  }

  struct SimplexHorizon
  {
    SimplexF* cf; // current face in the horizon
    SimplexF* ff; // first face in the horizon
    size_t nf; // number of faces in the horizon
    SimplexHorizon() : cf(NULL), ff(NULL), nf(0) {}
  };

private:
  unsigned int max_face_num;
  unsigned int max_vertex_num;
  unsigned int max_iterations;
  FCL_REAL tolerance;

public:

  enum Status {Valid, Touching, Degenerated, NonConvex, InvalidHull, OutOfFaces, OutOfVertices, AccuracyReached, FallBack, Failed};
  
  Status status;
  GJK::Simplex result;
  Vec3f normal;
  FCL_REAL depth;
  SimplexV* sv_store;
  SimplexF* fc_store;
  size_t nextsv;
  SimplexList hull, stock;

  EPA(unsigned int max_face_num_, unsigned int max_vertex_num_, unsigned int max_iterations_, FCL_REAL tolerance_) : max_face_num(max_face_num_),
                                                                                                                     max_vertex_num(max_vertex_num_),
                                                                                                                     max_iterations(max_iterations_),
                                                                                                                     tolerance(tolerance_)
  {
    initialize();
  }

  ~EPA()
  {
    delete [] sv_store;
    delete [] fc_store;
  }

  void initialize();

  bool getEdgeDist(SimplexF* face, SimplexV* a, SimplexV* b, FCL_REAL& dist);

  SimplexF* newFace(SimplexV* a, SimplexV* b, SimplexV* vertex, bool forced);

  /// @brief Find the best polytope face to split
  SimplexF* findBest();

  Status evaluate(GJK& gjk, const Vec3f& guess);

  /// @brief the goal is to add a face connecting vertex w and face edge f[e] 
  bool expand(size_t pass, SimplexV* w, SimplexF* f, size_t e, SimplexHorizon& horizon);  
};


} // details



}


} // namespace hpp

#endif
