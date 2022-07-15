#include <hpp/fcl/narrowphase/gjk.h>
#include <hpp/fcl/shape/convex.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include "hpp/fcl/mesh_loader/loader.h"
#include "hpp/fcl/BV/AABB.h"
#include "hpp/fcl/BVH/BVH_model.h"
#include "utility.h"

using hpp::fcl::BV_AABB;
using hpp::fcl::BVHModelPtr_t;
using hpp::fcl::ConvexBase;
using hpp::fcl::FCL_REAL;
using hpp::fcl::GJKConvergenceCriterion;
using hpp::fcl::GJKConvergenceCriterionType;
using hpp::fcl::GJKVariant;
using hpp::fcl::MeshLoader;
using hpp::fcl::NODE_TYPE;
using hpp::fcl::support_func_guess_t;
using hpp::fcl::Timer;
using hpp::fcl::Transform3f;
using hpp::fcl::Vec3f;
using hpp::fcl::details::GJK;
using hpp::fcl::details::MinkowskiDiff;

std::shared_ptr<ConvexBase> loadConvexShape(const std::string& file_name) {
  NODE_TYPE bv_type = BV_AABB;
  MeshLoader loader(bv_type);
  BVHModelPtr_t bvh1 = loader.load(file_name);
  bool _ = bvh1->buildConvexHull(true, "Qt");
  return bvh1->convex;
}

std::vector<Transform3f> generateRandomTransforms(int n_transfos) {
  FCL_REAL extents[] = {-1., -1., -1., 1., 1., 1.};
  std::vector<Transform3f> transforms;
  generateRandomTransforms(extents, transforms,
                           static_cast<size_t>(n_transfos));
  // The translations are set such that the shapes are not in contact.
  // This allows us to control the distance between the shapes afterwards.
  for (size_t i = 0; i < transforms.size(); i++) {
    Vec3f new_translation = transforms[i].getTranslation();
    new_translation.noalias() = 3 * new_translation / new_translation.norm();
    transforms[i].setTranslation(new_translation);
  }
  return transforms;
}

FCL_REAL evalSolver(GJK& solver, const ConvexBase* shape1,
                    const ConvexBase* shape2,
                    std::vector<Transform3f>& transforms, FCL_REAL distance) {
  // -- Pose shape 1
  Transform3f tf1 = Transform3f::Identity();

  // -- Minkowski difference and init guess/support
  MinkowskiDiff mink_diff;
  Vec3f init_guess;
  support_func_guess_t init_support_guess;
  Timer timer;

  // Solver used to separate the shapes at distance
  GJK separator(128, 1e-8);
  separator.convergence_criterion = GJKConvergenceCriterion::DualityGap;
  separator.convergence_criterion_type = GJKConvergenceCriterionType::Absolute;

  // -- Loop through all the poses
  FCL_REAL mean_time = 0.;
  FCL_REAL n_problem_solved = 1.;
  for (size_t i = 0; i < transforms.size(); i++) {
    Transform3f T = transforms[i];
    mink_diff.set(shape1, shape2, tf1, T);
    init_guess.noalias() =
        shape1->aabb_local.center() -
        (mink_diff.oR1 * shape2->aabb_local.center() + mink_diff.ot1);
    init_support_guess.setZero();

    GJK::Status status =
        separator.evaluate(mink_diff, init_guess, init_support_guess);
    // This allows us to set the distance that separates the shapes
    if (status == GJK::Valid && separator.ray.norm() > 0) {
      Vec3f normal = separator.ray / separator.ray.norm();
      Vec3f dt = separator.ray - distance * normal;
      Vec3f new_translation = T.getTranslation() + dt;
      T.setTranslation(new_translation);

      // We record the execution time
      FCL_REAL mean_time_problem = 0.;
      for (size_t j = 0; j < 100; j++) {
        timer.stop();
        timer.start();
        mink_diff.set(shape1, shape2, tf1, T);
        init_guess.noalias() =
            shape1->aabb_local.center() -
            (mink_diff.oR1 * shape2->aabb_local.center() + mink_diff.ot1);
        init_support_guess.setZero();
        status = solver.evaluate(mink_diff, init_guess, init_support_guess);
        timer.stop();
        mean_time_problem += timer.elapsed().user;
      }
      mean_time_problem /= 100;
      mean_time += mean_time_problem;
      n_problem_solved += 1;
    }
  }
  mean_time /= n_problem_solved;
  return mean_time;
}

int main() {
  const std::string& file_name_1 = "../test/benchmark/meshes/mesh1.stl";
  std::shared_ptr<ConvexBase> shape1 = loadConvexShape(file_name_1);
  std::cout << "Num vertices shape 1: " << shape1->num_points << std::endl;
  shape1->computeLocalAABB();

  const std::string& file_name_2 = "../test/benchmark/meshes/mesh2.dae";
  std::shared_ptr<ConvexBase> shape2 = loadConvexShape(file_name_2);
  std::cout << "Num vertices shape 2: " << shape2->num_points << std::endl;
  shape2->computeLocalAABB();

  // -- Generate random poses
  int n_transfos = 500;
  std::vector<Transform3f> transforms = generateRandomTransforms(n_transfos);

  // -- Solvers
  const unsigned int max_iterations = 128;
  const hpp::fcl::FCL_REAL tolerance = 1e-8;

  GJK gjk(max_iterations, tolerance);
  gjk.setDistanceEarlyBreak(0.);  // for boolean collision checking
  gjk.gjk_variant = GJKVariant::DefaultGJK;
  gjk.convergence_criterion = GJKConvergenceCriterion::DualityGap;
  gjk.convergence_criterion_type = GJKConvergenceCriterionType::Absolute;
  GJK nesterov(max_iterations, tolerance);
  nesterov.gjk_variant = GJKVariant::NesterovAcceleration;
  nesterov.setDistanceEarlyBreak(0.);  // for boolean collision checking
  nesterov.convergence_criterion = GJKConvergenceCriterion::DualityGap;
  nesterov.convergence_criterion_type = GJKConvergenceCriterionType::Absolute;

  // -- Distances between shapes
  std::vector<FCL_REAL> distances = {-0.1,  -0.01, -0.001, -0.0001, 0.0001,
                                     0.001, 0.01,  0.1,    1.};

  // -- Loop through all the distances
  for (size_t d = 0; d < distances.size(); d++) {
    FCL_REAL mean_time_gjk =
        evalSolver(gjk, shape1.get(), shape2.get(), transforms, distances[d]);
    FCL_REAL mean_time_nesterov = evalSolver(
        nesterov, shape1.get(), shape2.get(), transforms, distances[d]);
    std::cout << "---- Distance: " << distances[d] << " meters ----"
              << std::endl;
    std::cout << "Mean running time (in us)" << std::endl;
    std::cout << "    GJK:      " << mean_time_gjk << std::endl;
    std::cout << "    Nesterov: " << mean_time_nesterov << "\n" << std::endl;
  }
}
