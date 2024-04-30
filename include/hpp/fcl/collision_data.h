/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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
#include "hpp/fcl/logging.h"

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
  mutable Vec3f cached_gjk_guess;

  /// @brief the support function initial guess set by user
  mutable support_func_guess_t cached_support_func_guess;

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

  /// @brief Updates the guess for the internal GJK algorithm in order to
  /// warm-start it when reusing this collision request on the same collision
  /// pair.
  /// @note The option `gjk_initial_guess` must be set to
  /// `GJKInitialGuess::CachedGuess` for this to work.
  void updateGuess(const QueryResult& result) const;

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

inline void QueryRequest::updateGuess(const QueryResult& result) const {
  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  if (gjk_initial_guess == GJKInitialGuess::CachedGuess ||
      enable_cached_gjk_guess) {
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
  HPP_FCL_DEPRECATED_MESSAGE(A lower bound on distance is always computed.)
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

/// @brief This structure allows to encode support sets (sets of points which
/// are all support points in a certain direction).
/// This structure can also be used to represent a contact patch (see @ref
/// ContactPatch). A contact patch is the intersection of two support sets. We
/// will use contact patch and support set interchangeably throughout the
/// documentation. A support set/contact patch has a normal `n =
/// Contact::normal` and passes by point `p` (typically `p = Contact::pos`). If
/// we denote by P the plane passing by `p` and supported by `n`, a contact
/// patch is represented as a polytope which vertices all belong to `P & S1 &
/// S2`, where `&` denotes the set-intersection.
/// @note As of now (April 2024), the contact patch is a 2D contact surface in
/// 3D but internal algorithms of hpp-fcl can easily be extended to compute a
/// volume of contact instead.
/// In such a case, a contact volume could not be encoded as a `SupportSet`.
struct HPP_FCL_DLLAPI SupportSet {
 public:
  // clang-format off
  using Matrixx2fXpr = Eigen::Block<Matrixx2f, Eigen::Dynamic, 2, true>;
  using ConstMatrixx2fXpr = Eigen::Block<const Matrixx2f, Eigen::Dynamic, 2, true>;
  using Vec2fXpr = Eigen::Block<Matrixx2f, 1, 2, true>;
  using ConstVec2fXpr = Eigen::Block<const Matrixx2f, 1, 2, true>;
  using Index = Eigen::Index;
  // clang-format on

  /// @brief Reference frame in which to express a support points (contact point
  /// if SupportSet represents a ContactPatch).
  enum ReferenceFrame {
    // World frame, e.g. tfc is the support set frame (i.e contact frame),
    // expressed w.r.t
    // the world frame. Used to get the position of a contact point, expressed
    // in the world frame.
    WORLD = 0,
    // Local frame. Used to get the position of a point of
    // the set, expressed in the local frame of the set.
    LOCAL = 1,
    // TODO(louis): remove LOCAL_WORLD_ALIGNED
    // Local frame, but with axes aligned with those of the world frame.
    // Suppose we want to move the origin of the frame `this->tfc` to where the
    // i-th point is located, we can simply do:
    //   this->tfc.translation() += pi;
    // where pi is the LOCAL_WORLD_ALIGNED position of the i-th point.
    LOCAL_WORLD_ALIGNED = 2
  };

  /// @brief Set frame (i.e. contact frame), expressed in the world coordinates.
  /// @note The support set direction (i.e. contact normal) is the z-axis of the
  /// transform's rotation.
  Transform3f tfc;

  /// @brief Offset of the support set, i.e. penetration depth of the contact
  /// patch. This value corresponds to the signed distance between the shapes.
  /// @note If SupportSet is used as a ContactPatch, for each contact point `p`
  /// in the patch, `p1 = p + 0.5*d*n` and `p2 = p - 0.5*d*n` define a pair of
  /// witness points. `p1` belongs to the surface of the first shape, `p2`
  /// belongs to the surface of the second shape.
  FCL_REAL penetration_depth;

  /// @brief Default maximum size of the polytope representing the support set.
  static constexpr size_t default_max_size = 6;

 private:
  /// @brief Vertices of the polytope in the `SupportSet`.
  Matrixx2f m_points;

  /// @brief Current size of the support set.
  Index m_size;

 public:
  /// @brief Default constructor.
  explicit SupportSet(size_t max_size = default_max_size)
      : tfc(Transform3f::Identity()),
        penetration_depth(0),
        m_points(max_size, 2),
        m_size(0) {}

  /// @brief Maximum size of the support set.
  size_t capacity() const { return (size_t)(this->m_points.rows()); }

  /// @brief Current size of the support set.
  size_t size() const { return (size_t)(this->m_size); }

  /// @brief Update the capacity of the support set.
  /// @note This clears the content of the support set.
  void reserve(const size_t max_size) {
    this->m_points.resize((Index)max_size, 2);
    this->m_size = 0;
  }

  /// @brief Direction assiocated to the support set.
  /// If a `SupportSet` represents a `ContactPatch`, this is simply the normal
  /// of the patch.
  /// @note As in `Contact`, the normal always points from the first to the
  /// second shape.
  Vec3f getNormal() const { return this->tfc.rotation().col(2); }

  /// @brief Add a 2D point to the set.
  void addPoint(const Vec2f& point) {
    HPP_FCL_ASSERT(this->m_size < this->m_points.rows(),
                   "Tried to insert point in set but exceeded "
                   "maximum size of the set.",
                   std::logic_error);
    if (this->m_size < this->m_points.rows()) {
      this->m_points.row(this->m_size) = point;
      ++(this->m_size);
    } else {
      this->m_points.row(this->m_points.rows()) = point;
    }
  }

  /// @brief Add a 3D point to the set, expressed in the reference frame.
  /// @note This function takes a point and expresses it in the local frame of
  /// the support set. It then takes only the x and y components of the
  /// vector, effectively doing a projection onto the plane to which the set
  /// belongs.
  /// @tparam InputFrame is the reference frame in which the input 3D point is
  /// expressed. See @ref SupportSet::ReferenceFrame.
  template <int InputFrame = ReferenceFrame::WORLD>
  void addPoint(const Vec3f& point_3d) {
    if (InputFrame == ReferenceFrame::WORLD) {
      const auto point = this->tfc.inverseTransform(point_3d);
      this->addPoint(point.template head<2>());
    }
    if (InputFrame == ReferenceFrame::LOCAL) {
      this->addPoint(point_3d.template head<2>());
    }
    if (InputFrame == ReferenceFrame::LOCAL_WORLD_ALIGNED) {
      const auto point = this->tfc.rotation().transpose() * point_3d;
      this->addPoint(point.template head<2>());
    }
  }

  /// @brief Get the i-th point of the set, expressed in the 3D reference frame.
  /// @tparam OutputFrame is the reference frame in which the output 3D point is
  /// expressed. See @ref SupportSet::ReferenceFrame.
  template <int OutputFrame = ReferenceFrame::WORLD>
  Vec3f getPoint(const Index i) const {
    Vec3f point(0, 0, 0);
    point.head<2>() = this->point(i);
    if (OutputFrame == ReferenceFrame::WORLD) {
      point = tfc.transform(point);
    }
    if (OutputFrame == ReferenceFrame::LOCAL) {
      // do nothing
    }
    if (OutputFrame == ReferenceFrame::LOCAL_WORLD_ALIGNED) {
      point = tfc.rotation() * point;
    }
    return point;
  }

  /// @brief Getter for the 2D points in the support set.
  Matrixx2fXpr points() {
    HPP_FCL_ASSERT((this->m_size > 0) && (this->m_size < this->m_points.rows()),
                   "Invalid support set/contact patch size.", std::logic_error);
    return this->m_points.topRows(this->m_size);
  }

  /// @brief Const getter for the 2D points in the support set.
  ConstMatrixx2fXpr points() const {
    HPP_FCL_ASSERT((this->m_size > 0) && (this->m_size < this->m_points.rows()),
                   "Invalid support set/contact patch size.", std::logic_error);
    return this->m_points.topRows(this->m_size);
  }

  /// @brief Getter for the i-th 2D point in the support set.
  Vec2fXpr point(const Index i) {
    HPP_FCL_ASSERT((this->m_size > 0) && (this->m_size < this->m_points.rows()),
                   "Invalid support set/contact patch size.", std::logic_error);
    if (i < this->m_size) {
      return this->m_points.row(i);
    }
    return this->m_points.row(this->m_size);
  }

  /// @brief Const getter for the i-th 2D point in the support set.
  ConstVec2fXpr point(const Index i) const {
    HPP_FCL_ASSERT((this->m_size > 0) && (this->m_size < this->m_points.rows()),
                   "Invalid support set/contact patch size.", std::logic_error);
    if (i < this->m_size) {
      return this->m_points.row(i);
    }
    return this->m_points.row(this->m_size);
  }

  /// @brief Clear the support set.
  void clear() {
    this->m_size = 0;
    this->tfc.setIdentity();
    this->penetration_depth = 0;
  }

  /// @brief Reset the support set. Same effect as `clear` but does not modify
  /// `tfc` nor `penetration_depth`.
  void reset() { this->m_size = 0; }

  /// @brief Whether two support sets/contact patches are the same or not.
  /// @note This compares, term by term, two support sets.
  /// However, two support sets can be identical, but have a different order
  /// for their points. Use `isEqual` to compare two support sets.
  bool operator==(const SupportSet& other) const {
    return this->tfc == other.tfc &&
           this->penetration_depth == other.penetration_depth &&
           this->points() == other.points() &&
           this->capacity() == other.capacity();
  }

  /// @brief Whether two support sets are the same or not.
  /// Checks for different order of the points.
  bool isSame(const SupportSet& other,
              const FCL_REAL tol =
                  Eigen::NumTraits<FCL_REAL>::dummy_precision()) const {
    // The x and y axis of the set are arbitrary, but the z axis is
    // always the support direction/normal. The position of the origin of the
    // frame is also arbitrary. So we only check if the normals are the
    // same.
    if (!this->getNormal().isApprox(other.getNormal(), tol)) {
      return false;
    }

    if (std::abs(this->penetration_depth - other.penetration_depth) > tol) {
      return false;
    }

    if (this->size() != other.size()) {
      return false;
    }

    for (Index i = 0; i < (Index)(this->size()); ++i) {
      bool found = false;
      const Vec3f pi = this->getPoint<ReferenceFrame::WORLD>(i);
      for (Index j = 0; j < (Index)(this->size()); ++j) {
        const Vec3f other_pj = other.getPoint<ReferenceFrame::WORLD>(j);
        if (pi.isApprox(other_pj, tol)) {
          found = true;
        }
      }
      if (!found) {
        return false;
      }
    }

    return true;
  }
};

/// @brief A support set and a contact patch are identical things.
/// A contact patch is simply the intersection between two support sets: the
/// support set of shape S1 in direction `n` and the support set of shape S2 in
/// direction `-n`, where `n` is the contact normal (satisfying the optimality
/// conditions of GJK/EPA).
/// Therefore, a ContactPatch is still a support set.
/// @note A contact patch is **not** the support set of the Minkowski Difference
/// in the direction of the normal.
/// @note For now (April 2024), a `ContactPatch` is a 2D polytope,
/// so the points of the set, forming the convex-hull of the polytope, are
/// stored in a counter-clockwise fashion.
///
/// In details:
/// PART I - What is a contact point?
/// ---------------------------------
/// This section is important to understand what is a contact patch.
/// When calling `collide` on a collision pair, the `CollisionResult`
/// given as an input is populated with a vector of `Contact` (see @ref
/// CollisionResult and @ref Contact).
/// These structures encode the fact that S1 and S2 make contact at the
/// location `p = Contact::pos`, with normal `n = Contact::normal` and
/// penetration depth `d = Contact::penetration_depth`.
/// The tuple `(p, n, d)` has remarkable mathematical properties because we
/// always have:
///   - `p1 = p + 0.5*d*n` is a witness point, and belongs to the surface of S1,
///   - `p2 = p - 0.5*d*n` is a witness point, and belongs to the surface of S2.
///   - The pair `(p1, p2)` defines a pair of witness points such that
///   translating S1 by `p1 - p2` separates the shapes. The vector `v = p1 - p2`
///   is called a separating vector.
///   - Remarkably, if `w` is another separating vector, then **necessarily**
///   `||w|| >= ||v||`.
/// --> We call a "smallest separating vector" a separating vector `v` such that
/// `||v|| = |d|`. Any smallest separating vector `v` has a norm `||v|| = |d|`.
/// --> Finally, we call the "separating plane" `P` the plane supported by `n`,
/// and passing by `p`. This plane `P` can easily be translated by `+- v/2`;
/// this gives two planes tangent to the shapes S1 and S2, supported by `n` and
/// passing by `p1` and `p2` respectively.
///
/// PART II - What is a contact patch?
/// ----------------------------------
/// Up to this point, we have a normal which only maps to one contact point.
/// However, imagine a scenario where a box is sitting on a plane. Alghouth the
/// normal `n` is unique, the point `p` stored in `Contact::pos` and computed by
/// `collide` is arbitrary.
/// Hence, a contact patch is simply a surface (or volume) of contact between
/// two shapes S1 and S2. This contact surface is nothing more than the (triple)
/// intersection between the separating plane (see PART I), the shape S1 and the
/// shape S2, `P & S1 & S2`. If the contact patch is a volume, it is (an
/// approximation of) the intersection between S1 and S2.
/// TODO(louis): modify `computeContactPatch` to approximate the intersection
/// volume.
///
/// PART III - Can there be more than one contact patch?
/// ----------------------------------------------------
/// Yes, but again it is very rare in practice, as there needs to be more than
/// one smallest separation vector.
/// However, imagine two identical boxes (i.e. Box(1, 1, 1)) located at the
/// exact same pose in space. There are as many smallest separation vectors as
/// there are faces in a box, i.e. there are 6 smallest separation vectors. So
/// there can be at most 6 `hpp::fcl::Contact`. The HPP-FCL library implements
/// GJK and EPA and allows to recover up to `CollisionRequest::num_max_contacts`
/// smallest separation vectors. So in this scenario, we could recover the 6
/// normals (and contact position and penetration depth). Then, we can recover
/// the 6 contact patches.
/// TODO(louis): modify EPA to recover the entire optimal set.
using ContactPatch = SupportSet;

/// @brief Construct a frame from a `Contact`'s position and normal.
/// Because both `Contact`'s position and normal are expressed in the world
/// frame, this frame is also expressed w.r.t the world frame.
/// The origin of the frame is `contact.pos` and the z-axis of the frame is
/// `contact.normal`.
inline void constructContactPatchFrameFromContact(const Contact& contact,
                                                  ContactPatch& contact_patch) {
  contact_patch.penetration_depth = contact.penetration_depth;
  contact_patch.tfc.translation() = contact.pos;
  contact_patch.tfc.rotation() = constructBasisFromNormal(contact.normal);
}

/// @brief Request for a contact patch computation.
struct HPP_FCL_DLLAPI ContactPatchRequest {
 private:
  /// @brief The maximum number of contact patches that can be computed.
  size_t m_num_max_contact_patches{default_num_max_contact_patches};

  /// @brief Maximum number of vertices of each contact patch.
  size_t m_max_size_contact_patch{ContactPatch::default_max_size};

 public:
  /// @brief Default maximum number of contact patches requested.
  static constexpr size_t default_num_max_contact_patches = 1;

  /// @brief Get maximum number of requested contact patches.
  size_t getMaxNumContactPatch() const {
    return this->m_num_max_contact_patches;
  }

  /// @brief Set maximum number of requested contact patches.
  void setMaxNumContactPatch(const size_t max_num_contact_patches) {
    this->m_num_max_contact_patches = max_num_contact_patches;
    if (this->m_num_max_contact_patches == 0) {
      HPP_FCL_LOG_WARNING(
          "Created a ContactPatchRequest with "
          "`num_max_contact_patches = 0`. Setting `num_max_contact_patches` "
          "to default non-zero value to prevent bugs.");
      this->m_num_max_contact_patches = 1;
    }
  }

  /// @brief Get maximum size of requested contact patches.
  size_t getMaxSizeContactPatch() const {
    return this->m_max_size_contact_patch;
  }

  /// @brief Set maximum size of requested contact patches.
  void setMaxSizeContactPatch(const size_t max_size_contact_patch) {
    this->m_max_size_contact_patch = max_size_contact_patch;
    if (this->m_max_size_contact_patch == 0) {
      HPP_FCL_LOG_WARNING(
          "Warning, created a ContactPatchRequest with "
          "`max_size_contact_patch = 0`. Setting `max_size_contact_patch` "
          "to default non-zero value to prevent bugs.");
      this->m_max_size_contact_patch = ContactPatch::default_max_size;
    }
  }

  explicit ContactPatchRequest(size_t max_num_contact_patches = 1,
                               size_t max_size_contact_patch = 6) {
    this->setMaxNumContactPatch(max_num_contact_patches);
    this->setMaxSizeContactPatch(max_size_contact_patch);
  }
};

/// @brief Result for a contact patch computation.
struct HPP_FCL_DLLAPI ContactPatchResult {
  using ContactPatchVector = std::vector<ContactPatch>;
  using ContactPatchRef = std::reference_wrapper<ContactPatch>;
  using ContactPatchRefVector = std::vector<ContactPatchRef>;

 private:
  /// @brief Data container for the vector of contact patches.
  /// @note Contrary to `CollisionResult` or `DistanceResult`, which have a very
  /// small memory footprint, contact patches can contain relatively large
  /// polytopes. In order to reuse a `ContactPatchResult` while avoiding
  /// successive mallocs, we have a data container and a vector which points to
  /// the currently active patches in this data container.
  ContactPatchVector m_contact_patches_data;

  /// @brief Contact patches in `m_contact_patches_data` can have two statuses:
  /// used or unused. This index tracks the first unused patch in the
  /// `m_contact_patches_data` vector.
  size_t m_id_available_patch;

  /// @brief Vector of contact patches of the result.
  ContactPatchRefVector m_contact_patches;

 public:
  /// @brief Default constructor.
  ContactPatchResult() : m_id_available_patch(0) {
    const size_t max_num_contact_patches = 1;
    const ContactPatchRequest request(max_num_contact_patches);
    this->set(request);
  }

  /// @brief Constructor using a `ContactPatchRequest`.
  explicit ContactPatchResult(const ContactPatchRequest& request)
      : m_id_available_patch(0) {
    this->set(request);
  };

  /// @brief Number of contact patches in the result.
  size_t numContactPatches() const { return this->m_contact_patches.size(); }

  /// @brief Maximum number of contact patches the result can store.
  size_t maxNumContactPatches() const {
    return this->m_contact_patches_data.size();
  }

  /// @brief Returns a new unused contact patch from the internal data vector.
  /// @note If there are no more unused contact patches, this method will return
  /// the last element of the internal data vector.
  ContactPatchRef getUnusedContactPatch() {
    if (this->m_id_available_patch >= this->m_contact_patches_data.size()) {
      HPP_FCL_LOG_WARNING(
          "Trying to get an unused contact patch but all contact patches are "
          "used. Getting the last used contact patch as fallback.");
    }
    if (this->m_id_available_patch < this->m_contact_patches_data.size()) {
      ContactPatch& contact_patch =
          this->m_contact_patches_data[this->m_id_available_patch];
      this->m_contact_patches.emplace_back(contact_patch);
      ++(this->m_id_available_patch);
    }
    return this->m_contact_patches.back();
  }

  /// @brief Getter for the i-th contact patch of the result.
  ContactPatch& getContactPatch(const size_t i) {
    if (this->m_contact_patches.empty()) {
      HPP_FCL_THROW_PRETTY(
          "The number of contact patches is zero. No ContactPatch can be "
          "returned.",
          std::invalid_argument);
    }
    if (i < this->m_contact_patches.size()) {
      return this->m_contact_patches[i];
    }
    return this->m_contact_patches.back();
  }

  /// @brief Const getter for the i-th contact patch of the result.
  const ContactPatch& getContactPatch(const size_t i) const {
    if (this->m_contact_patches.empty()) {
      HPP_FCL_THROW_PRETTY(
          "The number of contact patches is zero. No ContactPatch can be "
          "returned.",
          std::invalid_argument);
    }
    if (i < this->m_contact_patches.size()) {
      return this->m_contact_patches[i];
    }
    return this->m_contact_patches.back();
  }

  /// @brief Clears the contact patch result.
  void clear() {
    this->m_contact_patches.clear();
    this->m_id_available_patch = 0;
    for (ContactPatch& patch : this->m_contact_patches_data) {
      patch.clear();
    }
  }

  /// @brief Set up a `ContactPatchResult` from a `ContactPatchRequest`
  void set(const ContactPatchRequest& request) {
    HPP_FCL_ASSERT(request.getMaxNumContactPatch() > 0,
                   "The ContactPatchRequest has 0 max_num_contact_patches.",
                   std::logic_error);
    HPP_FCL_ASSERT(request.getMaxSizeContactPatch() > 0,
                   "The ContactPatchRequest has 0 max_size_contact_patch.",
                   std::logic_error);
    if (this->m_contact_patches_data.size() < request.getMaxNumContactPatch()) {
      this->m_contact_patches_data.resize(request.getMaxNumContactPatch());
    }
    for (ContactPatch& patch : this->m_contact_patches_data) {
      patch.reserve(request.getMaxSizeContactPatch());
    }
    this->clear();
  }

  /// @brief Return true if this `ContactPatchResult` is aligned with the
  /// `ContactPatchRequest` given as input.
  bool check(const ContactPatchRequest& request) const {
    assert(this->m_contact_patches_data.size() >=
           request.getMaxNumContactPatch());
    if (this->m_contact_patches_data.size() < request.getMaxNumContactPatch()) {
      return false;
    }

    for (const ContactPatch& patch : this->m_contact_patches_data) {
      if (patch.capacity() < request.getMaxSizeContactPatch()) {
        assert(patch.capacity() >= request.getMaxSizeContactPatch());
        return false;
      }
    }
    return true;
  }
};

struct DistanceResult;

/// @brief request to the distance computation
struct HPP_FCL_DLLAPI DistanceRequest : QueryRequest {
  /// @brief whether to return the nearest points.
  /// Nearest points are always computed and are the points of the shapes that
  /// achieve a distance of `DistanceResult::min_distance`.
  HPP_FCL_DEPRECATED_MESSAGE(
      Nearest points are always computed
      : they are the points of the shapes that achieve a distance of
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
