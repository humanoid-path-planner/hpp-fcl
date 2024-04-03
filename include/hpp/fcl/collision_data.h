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

#ifndef HPP_FCL_COLLISION_DATA_H
#define HPP_FCL_COLLISION_DATA_H

#include <vector>
#include <array>
#include <set>
#include <limits>

#include "hpp/fcl/collision_object.h"
#include "hpp/fcl/config.hh"
#include "hpp/fcl/data_types.h"
#include "hpp/fcl/timings.h"
#include "hpp/fcl/narrowphase/narrowphase_defaults.h"

namespace hpp {
namespace fcl {

/// @brief Contact information returned by collision
struct HPP_FCL_DLLAPI Contact {
  /// @brief collision object 1
  const CollisionGeometry* o1;

  /// @brief collision object 2
  const CollisionGeometry* o2;

  /// @brief contact primitive in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;

  /// @brief contact primitive in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;

  /// @brief contact normal, pointing from o1 to o2.
  /// The normal defined as the normalized separation vector:
  /// normal = (p2 - p1) / dist(o1, o2), where p1 = nearest_points[0]
  /// belongs to o1 and p2 = nearest_points[1] belongs to o2 and dist(o1, o2) is
  /// the **signed** distance between o1 and o2. The normal always points from
  /// o1 to o2.
  /// @note The separation vector is the smallest vector such that if o1 is
  /// translated by it, o1 and o2 are in touching contact (they share at least
  /// one contact point but have a zero intersection volume). If the shapes
  /// overlap, dist(o1, o2) = -((p2-p1).norm()). Otherwise, dist(o1, o2) =
  /// (p2-p1).norm().
  Vec3f normal;

  /// @brief nearest points associated to this contact.
  /// @note Also referred as "witness points" in other collision libraries.
  /// The points p1 = nearest_points[0] and p2 = nearest_points[1] verify the
  /// property that dist(o1, o2) * (p1 - p2) is the separation vector between o1
  /// and o2, with dist(o1, o2) being the **signed** distance separating o1 from
  /// o2. See \ref DistanceResult::normal for the definition of the separation
  /// vector. If o1 and o2 have multiple contacts, the nearest_points are
  /// associated with the contact which has the greatest penetration depth.
  /// TODO (louis): rename `nearest_points` to `witness_points`.
  std::array<Vec3f, 2> nearest_points;

  /// @brief contact position, in world space
  Vec3f pos;

  /// @brief penetration depth
  FCL_REAL penetration_depth;

  /// @brief invalid contact primitive information
  static const int NONE = -1;

  /// @brief Default constructor
  Contact() : o1(NULL), o2(NULL), b1(NONE), b2(NONE) {
    penetration_depth = (std::numeric_limits<FCL_REAL>::max)();
    nearest_points[0] = nearest_points[1] = normal = pos =
        Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN());
  }

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_,
          int b2_)
      : o1(o1_), o2(o2_), b1(b1_), b2(b2_) {
    penetration_depth = (std::numeric_limits<FCL_REAL>::max)();
    nearest_points[0] = nearest_points[1] = normal = pos =
        Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN());
  }

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_,
          int b2_, const Vec3f& pos_, const Vec3f& normal_, FCL_REAL depth_)
      : o1(o1_),
        o2(o2_),
        b1(b1_),
        b2(b2_),
        normal(normal_),
        nearest_points{pos_ - (depth_ * normal_ / 2),
                       pos_ + (depth_ * normal_ / 2)},
        pos(pos_),
        penetration_depth(depth_) {}

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_,
          int b2_, const Vec3f& p1, const Vec3f& p2, const Vec3f& normal_,
          FCL_REAL depth_)
      : o1(o1_),
        o2(o2_),
        b1(b1_),
        b2(b2_),
        normal(normal_),
        nearest_points{p1, p2},
        pos((p1 + p2) / 2),
        penetration_depth(depth_) {}

  bool operator<(const Contact& other) const {
    if (b1 == other.b1) return b2 < other.b2;
    return b1 < other.b1;
  }

  bool operator==(const Contact& other) const {
    return o1 == other.o1 && o2 == other.o2 && b1 == other.b1 &&
           b2 == other.b2 && normal == other.normal && pos == other.pos &&
           nearest_points[0] == other.nearest_points[0] &&
           nearest_points[1] == other.nearest_points[1] &&
           penetration_depth == other.penetration_depth;
  }

  bool operator!=(const Contact& other) const { return !(*this == other); }

  FCL_REAL getDistanceToCollision(const CollisionRequest& request) const;
};

struct QueryResult;

/// @brief base class for all query requests
struct HPP_FCL_DLLAPI QueryRequest {
  // @brief Initial guess to use for the GJK algorithm
  GJKInitialGuess gjk_initial_guess;

  /// @brief whether enable gjk initial guess
  /// @Deprecated Use gjk_initial_guess instead
  HPP_FCL_DEPRECATED_MESSAGE(Use gjk_initial_guess instead)
  bool enable_cached_gjk_guess;

  /// @brief the gjk initial guess set by user
  Vec3f cached_gjk_guess;

  /// @brief the support function initial guess set by user
  support_func_guess_t cached_support_func_guess;

  /// @brief maximum iteration for the GJK algorithm
  size_t gjk_max_iterations;

  /// @brief tolerance for the GJK algorithm.
  /// Note: This tolerance determines the precision on the estimated distance
  /// between two geometries which are not in collision.
  /// It is recommended to not set this tolerance to less than 1e-6.
  FCL_REAL gjk_tolerance;

  /// @brief whether to enable the Nesterov accleration of GJK
  GJKVariant gjk_variant;

  /// @brief convergence criterion used to stop GJK
  GJKConvergenceCriterion gjk_convergence_criterion;

  /// @brief convergence criterion used to stop GJK
  GJKConvergenceCriterionType gjk_convergence_criterion_type;

  /// @brief max number of iterations for EPA
  size_t epa_max_iterations;

  /// @brief tolerance for EPA.
  /// Note: This tolerance determines the precision on the estimated distance
  /// between two geometries which are in collision.
  /// It is recommended to not set this tolerance to less than 1e-6.
  /// Also, setting EPA's tolerance to less than GJK's is not recommended.
  FCL_REAL epa_tolerance;

  /// @brief enable timings when performing collision/distance request
  bool enable_timings;

  /// @brief threshold below which a collision is considered.
  FCL_REAL collision_distance_threshold;

  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  /// @brief Default constructor.
  QueryRequest()
      : gjk_initial_guess(GJKInitialGuess::DefaultGuess),
        enable_cached_gjk_guess(false),
        cached_gjk_guess(1, 0, 0),
        cached_support_func_guess(support_func_guess_t::Zero()),
        gjk_max_iterations(GJK_DEFAULT_MAX_ITERATIONS),
        gjk_tolerance(GJK_DEFAULT_TOLERANCE),
        gjk_variant(GJKVariant::DefaultGJK),
        gjk_convergence_criterion(GJKConvergenceCriterion::Default),
        gjk_convergence_criterion_type(GJKConvergenceCriterionType::Relative),
        epa_max_iterations(EPA_DEFAULT_MAX_ITERATIONS),
        epa_tolerance(EPA_DEFAULT_TOLERANCE),
        enable_timings(false),
        collision_distance_threshold(
            Eigen::NumTraits<FCL_REAL>::dummy_precision()) {}

  /// @brief Copy  constructor.
  QueryRequest(const QueryRequest& other) = default;

  /// @brief Copy  assignment operator.
  QueryRequest& operator=(const QueryRequest& other) = default;
  HPP_FCL_COMPILER_DIAGNOSTIC_POP

  void updateGuess(const QueryResult& result);

  /// @brief whether two QueryRequest are the same or not
  inline bool operator==(const QueryRequest& other) const {
    HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
    HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    return gjk_initial_guess == other.gjk_initial_guess &&
           enable_cached_gjk_guess == other.enable_cached_gjk_guess &&
           gjk_variant == other.gjk_variant &&
           gjk_convergence_criterion == other.gjk_convergence_criterion &&
           gjk_convergence_criterion_type ==
               other.gjk_convergence_criterion_type &&
           gjk_tolerance == other.gjk_tolerance &&
           gjk_max_iterations == other.gjk_max_iterations &&
           cached_gjk_guess == other.cached_gjk_guess &&
           cached_support_func_guess == other.cached_support_func_guess &&
           epa_max_iterations == other.epa_max_iterations &&
           epa_tolerance == other.epa_tolerance &&
           enable_timings == other.enable_timings &&
           collision_distance_threshold == other.collision_distance_threshold;
    HPP_FCL_COMPILER_DIAGNOSTIC_POP
  }
};

/// @brief base class for all query results
struct HPP_FCL_DLLAPI QueryResult {
  /// @brief stores the last GJK ray when relevant.
  Vec3f cached_gjk_guess;

  /// @brief stores the last support function vertex index, when relevant.
  support_func_guess_t cached_support_func_guess;

  /// @brief timings for the given request
  CPUTimes timings;

  QueryResult()
      : cached_gjk_guess(Vec3f::Zero()),
        cached_support_func_guess(support_func_guess_t::Constant(-1)) {}
};

inline void QueryRequest::updateGuess(const QueryResult& result) {
  if (gjk_initial_guess == GJKInitialGuess::CachedGuess) {
    cached_gjk_guess = result.cached_gjk_guess;
    cached_support_func_guess = result.cached_support_func_guess;
  }
  // TODO: use gjk_initial_guess instead
  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  if (enable_cached_gjk_guess) {
    cached_gjk_guess = result.cached_gjk_guess;
    cached_support_func_guess = result.cached_support_func_guess;
  }
  HPP_FCL_COMPILER_DIAGNOSTIC_POP
}

struct CollisionResult;

/// @brief flag declaration for specifying required params in CollisionResult
enum CollisionRequestFlag {
  CONTACT = 0x00001,
  DISTANCE_LOWER_BOUND = 0x00002,
  NO_REQUEST = 0x01000
};

/// @brief request to the collision algorithm
struct HPP_FCL_DLLAPI CollisionRequest : QueryRequest {
  /// @brief The maximum number of contacts that can be returned
  size_t num_max_contacts;

  /// @brief whether the contact information (normal, penetration depth and
  /// contact position) will return.
  bool enable_contact;

  /// Whether a lower bound on distance is returned when objects are disjoint
  HPP_FCL_DEPRECATED_MESSAGE(
      `enable_distance_lower_bound` is deprecated
           .A lower bound on distance is always computed.)
  bool enable_distance_lower_bound;

  /// @brief Distance below which objects are considered in collision.
  /// See \ref hpp_fcl_collision_and_distance_lower_bound_computation
  /// @note If set to -inf, the objects tested for collision are considered
  ///       as collision free and no test is actually performed by functions
  ///       hpp::fcl::collide of class hpp::fcl::ComputeCollision.
  FCL_REAL security_margin;

  /// @brief Distance below which bounding volumes are broken down.
  /// See \ref hpp_fcl_collision_and_distance_lower_bound_computation
  FCL_REAL break_distance;

  /// @brief Distance above which GJK solver makes an early stopping.
  /// GJK stops searching for the closest points when it proves that the
  /// distance between two geometries is above this threshold.
  ///
  /// @remarks Consequently, the closest points might be incorrect, but allows
  /// to save computational resources.
  FCL_REAL distance_upper_bound;

  /// @brief Constructor from a flag and a maximal number of contacts.
  ///
  /// @param[in] flag Collision request flag
  /// @param[in] num_max_contacts  Maximal number of allowed contacts
  ///
  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  CollisionRequest(const CollisionRequestFlag flag, size_t num_max_contacts_)
      : num_max_contacts(num_max_contacts_),
        enable_contact(flag & CONTACT),
        enable_distance_lower_bound(flag & DISTANCE_LOWER_BOUND),
        security_margin(0),
        break_distance(1e-3),
        distance_upper_bound((std::numeric_limits<FCL_REAL>::max)()) {}

  /// @brief Default constructor.
  CollisionRequest()
      : num_max_contacts(1),
        enable_contact(true),
        enable_distance_lower_bound(false),
        security_margin(0),
        break_distance(1e-3),
        distance_upper_bound((std::numeric_limits<FCL_REAL>::max)()) {}
  HPP_FCL_COMPILER_DIAGNOSTIC_POP

  bool isSatisfied(const CollisionResult& result) const;

  /// @brief whether two CollisionRequest are the same or not
  inline bool operator==(const CollisionRequest& other) const {
    HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
    HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    return QueryRequest::operator==(other) &&
           num_max_contacts == other.num_max_contacts &&
           enable_contact == other.enable_contact &&
           enable_distance_lower_bound == other.enable_distance_lower_bound &&
           security_margin == other.security_margin &&
           break_distance == other.break_distance &&
           distance_upper_bound == other.distance_upper_bound;
    HPP_FCL_COMPILER_DIAGNOSTIC_POP
  }
};

inline FCL_REAL Contact::getDistanceToCollision(
    const CollisionRequest& request) const {
  return penetration_depth - request.security_margin;
}

/// @brief collision result
struct HPP_FCL_DLLAPI CollisionResult : QueryResult {
 private:
  /// @brief contact information
  std::vector<Contact> contacts;

 public:
  /// Lower bound on distance between objects if they are disjoint.
  /// See \ref hpp_fcl_collision_and_distance_lower_bound_computation
  /// @note Always computed. If \ref CollisionRequest::distance_upper_bound is
  /// set to infinity, distance_lower_bound is the actual distance between the
  /// shapes.
  FCL_REAL distance_lower_bound;

  /// @brief normal associated to nearest_points.
  /// Same as `CollisionResult::nearest_points` but for the normal.
  Vec3f normal;

  /// @brief nearest points.
  /// A `CollisionResult` can have multiple contacts.
  /// The nearest points in `CollisionResults` correspond to the witness points
  /// associated with the smallest distance i.e the `distance_lower_bound`.
  /// For bounding volumes and BVHs, these nearest points are available
  /// only when distance_lower_bound is inferior to
  /// CollisionRequest::break_distance.
  std::array<Vec3f, 2> nearest_points;

 public:
  CollisionResult()
      : distance_lower_bound((std::numeric_limits<FCL_REAL>::max)()) {
    nearest_points[0] = nearest_points[1] = normal =
        Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN());
  }

  /// @brief Update the lower bound only if the distance is inferior.
  inline void updateDistanceLowerBound(const FCL_REAL& distance_lower_bound_) {
    if (distance_lower_bound_ < distance_lower_bound)
      distance_lower_bound = distance_lower_bound_;
  }

  /// @brief add one contact into result structure
  inline void addContact(const Contact& c) { contacts.push_back(c); }

  /// @brief whether two CollisionResult are the same or not
  inline bool operator==(const CollisionResult& other) const {
    return contacts == other.contacts &&
           distance_lower_bound == other.distance_lower_bound &&
           nearest_points[0] == other.nearest_points[0] &&
           nearest_points[1] == other.nearest_points[1] &&
           normal == other.normal;
  }

  /// @brief return binary collision result
  bool isCollision() const { return contacts.size() > 0; }

  /// @brief number of contacts found
  size_t numContacts() const { return contacts.size(); }

  /// @brief get the i-th contact calculated
  const Contact& getContact(size_t i) const {
    if (contacts.size() == 0)
      HPP_FCL_THROW_PRETTY(
          "The number of contacts is zero. No Contact can be returned.",
          std::invalid_argument);

    if (i < contacts.size())
      return contacts[i];
    else
      return contacts.back();
  }

  /// @brief set the i-th contact calculated
  void setContact(size_t i, const Contact& c) {
    if (contacts.size() == 0)
      HPP_FCL_THROW_PRETTY(
          "The number of contacts is zero. No Contact can be returned.",
          std::invalid_argument);

    if (i < contacts.size())
      contacts[i] = c;
    else
      contacts.back() = c;
  }

  /// @brief get all the contacts
  void getContacts(std::vector<Contact>& contacts_) const {
    contacts_.resize(contacts.size());
    std::copy(contacts.begin(), contacts.end(), contacts_.begin());
  }

  const std::vector<Contact>& getContacts() const { return contacts; }

  /// @brief clear the results obtained
  void clear() {
    distance_lower_bound = (std::numeric_limits<FCL_REAL>::max)();
    contacts.clear();
    timings.clear();
    nearest_points[0] = nearest_points[1] = normal =
        Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN());
  }

  /// @brief reposition Contact objects when fcl inverts them
  /// during their construction.
  void swapObjects();
};

struct DistanceResult;

/// @brief request to the distance computation
struct HPP_FCL_DLLAPI DistanceRequest : QueryRequest {
  /// @brief whether to return the nearest points.
  /// Nearest points are always computed and are the points of the shapes that
  /// achieve a distance of `DistanceResult::min_distance`.
  HPP_FCL_DEPRECATED_MESSAGE(
      `enable_nearest_points` is deprecated.Nearest points are always computed;
       they are the points of the shapes that achieve a distance of
      `DistanceResult::min_distance`
           .\n Use `enable_signed_distance` if you want to compute a signed
               minimum distance(and thus its corresponding nearest points)
           .)
  bool enable_nearest_points;

  /// @brief whether to compute the penetration depth when objects are in
  /// collision.
  /// Turning this off can save computation time if only the distance
  /// when objects are disjoint is needed.
  /// @note The minimum distance between the shapes is stored in
  /// `DistanceResult::min_distance`.
  /// If `enable_signed_distance` is off, `DistanceResult::min_distance`
  /// is always positive.
  /// If `enable_signed_distance` is on, `DistanceResult::min_distance`
  /// can be positive or negative.
  /// The nearest points are the points of the shapes that achieve
  /// a distance of `DistanceResult::min_distance`.
  bool enable_signed_distance;

  /// @brief error threshold for approximate distance
  FCL_REAL rel_err;  // relative error, between 0 and 1
  FCL_REAL abs_err;  // absolute error

  /// \param enable_nearest_points_ enables the nearest points computation.
  /// \param enable_signed_distance_ allows to compute the penetration depth
  /// \param rel_err_
  /// \param abs_err_
  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  DistanceRequest(bool enable_nearest_points_ = true,
                  bool enable_signed_distance_ = true, FCL_REAL rel_err_ = 0.0,
                  FCL_REAL abs_err_ = 0.0)
      : enable_nearest_points(enable_nearest_points_),
        enable_signed_distance(enable_signed_distance_),
        rel_err(rel_err_),
        abs_err(abs_err_) {}
  HPP_FCL_COMPILER_DIAGNOSTIC_POP

  bool isSatisfied(const DistanceResult& result) const;

  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  DistanceRequest& operator=(const DistanceRequest& other) = default;
  HPP_FCL_COMPILER_DIAGNOSTIC_POP

  /// @brief whether two DistanceRequest are the same or not
  inline bool operator==(const DistanceRequest& other) const {
    HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
    HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    return QueryRequest::operator==(other) &&
           enable_nearest_points == other.enable_nearest_points &&
           enable_signed_distance == other.enable_signed_distance &&
           rel_err == other.rel_err && abs_err == other.abs_err;
    HPP_FCL_COMPILER_DIAGNOSTIC_POP
  }
};

/// @brief distance result
struct HPP_FCL_DLLAPI DistanceResult : QueryResult {
 public:
  /// @brief minimum distance between two objects.
  /// If two objects are in collision and
  /// DistanceRequest::enable_signed_distance is activated, min_distance <= 0.
  /// @note The nearest points are the points of the shapes that achieve a
  /// distance of `DistanceResult::min_distance`.
  FCL_REAL min_distance;

  /// @brief normal.
  Vec3f normal;

  /// @brief nearest points.
  /// See CollisionResult::nearest_points.
  std::array<Vec3f, 2> nearest_points;

  /// @brief collision object 1
  const CollisionGeometry* o1;

  /// @brief collision object 2
  const CollisionGeometry* o2;

  /// @brief information about the nearest point in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;

  /// @brief information about the nearest point in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;

  /// @brief invalid contact primitive information
  static const int NONE = -1;

  DistanceResult(
      FCL_REAL min_distance_ = (std::numeric_limits<FCL_REAL>::max)())
      : min_distance(min_distance_), o1(NULL), o2(NULL), b1(NONE), b2(NONE) {
    const Vec3f nan(
        Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN()));
    nearest_points[0] = nearest_points[1] = normal = nan;
  }

  /// @brief add distance information into the result
  void update(FCL_REAL distance, const CollisionGeometry* o1_,
              const CollisionGeometry* o2_, int b1_, int b2_) {
    if (min_distance > distance) {
      min_distance = distance;
      o1 = o1_;
      o2 = o2_;
      b1 = b1_;
      b2 = b2_;
    }
  }

  /// @brief add distance information into the result
  void update(FCL_REAL distance, const CollisionGeometry* o1_,
              const CollisionGeometry* o2_, int b1_, int b2_, const Vec3f& p1,
              const Vec3f& p2, const Vec3f& normal_) {
    if (min_distance > distance) {
      min_distance = distance;
      o1 = o1_;
      o2 = o2_;
      b1 = b1_;
      b2 = b2_;
      nearest_points[0] = p1;
      nearest_points[1] = p2;
      normal = normal_;
    }
  }

  /// @brief add distance information into the result
  void update(const DistanceResult& other_result) {
    if (min_distance > other_result.min_distance) {
      min_distance = other_result.min_distance;
      o1 = other_result.o1;
      o2 = other_result.o2;
      b1 = other_result.b1;
      b2 = other_result.b2;
      nearest_points[0] = other_result.nearest_points[0];
      nearest_points[1] = other_result.nearest_points[1];
      normal = other_result.normal;
    }
  }

  /// @brief clear the result
  void clear() {
    const Vec3f nan(
        Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN()));
    min_distance = (std::numeric_limits<FCL_REAL>::max)();
    o1 = NULL;
    o2 = NULL;
    b1 = NONE;
    b2 = NONE;
    nearest_points[0] = nearest_points[1] = normal = nan;
    timings.clear();
  }

  /// @brief whether two DistanceResult are the same or not
  inline bool operator==(const DistanceResult& other) const {
    bool is_same = min_distance == other.min_distance &&
                   nearest_points[0] == other.nearest_points[0] &&
                   nearest_points[1] == other.nearest_points[1] &&
                   normal == other.normal && o1 == other.o1 && o2 == other.o2 &&
                   b1 == other.b1 && b2 == other.b2;

    // TODO: check also that two GeometryObject are indeed equal.
    if ((o1 != NULL) ^ (other.o1 != NULL)) return false;
    is_same &= (o1 == other.o1);
    //    else if (o1 != NULL and other.o1 != NULL) is_same &= *o1 == *other.o1;

    if ((o2 != NULL) ^ (other.o2 != NULL)) return false;
    is_same &= (o2 == other.o2);
    //    else if (o2 != NULL and other.o2 != NULL) is_same &= *o2 == *other.o2;

    return is_same;
  }
};

namespace internal {
inline void updateDistanceLowerBoundFromBV(const CollisionRequest& /*req*/,
                                           CollisionResult& res,
                                           const FCL_REAL sqrDistLowerBound) {
  // BV cannot find negative distance.
  if (res.distance_lower_bound <= 0) return;
  FCL_REAL new_dlb = std::sqrt(sqrDistLowerBound);  // - req.security_margin;
  if (new_dlb < res.distance_lower_bound) res.distance_lower_bound = new_dlb;
}

inline void updateDistanceLowerBoundFromLeaf(const CollisionRequest&,
                                             CollisionResult& res,
                                             const FCL_REAL& distance,
                                             const Vec3f& p0, const Vec3f& p1,
                                             const Vec3f& normal) {
  if (distance < res.distance_lower_bound) {
    res.distance_lower_bound = distance;
    res.nearest_points[0] = p0;
    res.nearest_points[1] = p1;
    res.normal = normal;
  }
}
}  // namespace internal

inline CollisionRequestFlag operator~(CollisionRequestFlag a) {
  return static_cast<CollisionRequestFlag>(~static_cast<int>(a));
}

inline CollisionRequestFlag operator|(CollisionRequestFlag a,
                                      CollisionRequestFlag b) {
  return static_cast<CollisionRequestFlag>(static_cast<int>(a) |
                                           static_cast<int>(b));
}

inline CollisionRequestFlag operator&(CollisionRequestFlag a,
                                      CollisionRequestFlag b) {
  return static_cast<CollisionRequestFlag>(static_cast<int>(a) &
                                           static_cast<int>(b));
}

inline CollisionRequestFlag operator^(CollisionRequestFlag a,
                                      CollisionRequestFlag b) {
  return static_cast<CollisionRequestFlag>(static_cast<int>(a) ^
                                           static_cast<int>(b));
}

inline CollisionRequestFlag& operator|=(CollisionRequestFlag& a,
                                        CollisionRequestFlag b) {
  return (CollisionRequestFlag&)((int&)(a) |= static_cast<int>(b));
}

inline CollisionRequestFlag& operator&=(CollisionRequestFlag& a,
                                        CollisionRequestFlag b) {
  return (CollisionRequestFlag&)((int&)(a) &= static_cast<int>(b));
}

inline CollisionRequestFlag& operator^=(CollisionRequestFlag& a,
                                        CollisionRequestFlag b) {
  return (CollisionRequestFlag&)((int&)(a) ^= static_cast<int>(b));
}

}  // namespace fcl

}  // namespace hpp

#endif
