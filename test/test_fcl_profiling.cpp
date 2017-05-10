// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-fcl.
// hpp-fcl is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-fcl is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-fcl. If not, see <http://www.gnu.org/licenses/>.


#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/collision_func_matrix.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"

using namespace fcl;

CollisionFunctionMatrix<GJKSolver_indep> lookupTable;
bool supportedPair(const CollisionGeometry* o1, const CollisionGeometry* o2)
{
  OBJECT_TYPE object_type1 = o1->getObjectType();
  OBJECT_TYPE object_type2 = o2->getObjectType();
  NODE_TYPE node_type1 = o1->getNodeType();
  NODE_TYPE node_type2 = o2->getNodeType();

  if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
    return (lookupTable.collision_matrix[node_type2][node_type1] != NULL);
  else
    return (lookupTable.collision_matrix[node_type1][node_type2] != NULL);
}

template <typename BV/*, SplitMethodType split_method*/>
CollisionGeometryPtr_t objToGeom (const std::string& filename)
{
  std::vector<Vec3f> pt;
  std::vector<Triangle> tri;
  loadOBJFile(filename.c_str(), pt, tri);

  BVHModel<BV>* model (new BVHModel<BV>());
  // model->bv_splitter.reset(new BVSplitter<BV>(split_method));

  model->beginModel();
  model->addSubModel(pt, tri);
  model->endModel();
  
  return CollisionGeometryPtr_t (model);
}

struct Geometry {
  std::string type;
  CollisionGeometryPtr_t o;

  Geometry (const std::string& t, CollisionGeometry* ob) : 
    type(t), o(ob)
  {}
  Geometry (const std::string& t, const CollisionGeometryPtr_t& ob) : 
    type(t), o(ob)
  {}
};

// struct Result {
  // CollisionResult r;
  // double time_us;
// };

struct Results {
  std::vector<CollisionResult> rs;
  Eigen::VectorXd times; // micro-seconds

  Results (int i) : rs(i), times(i) {}
};

void collide(const std::vector<Transform3f>& tf,
             const CollisionGeometry* o1,
             const CollisionGeometry* o2,
             const CollisionRequest& request,
             Results& results)
{
  Transform3f Id;
  Timer timer;
  for (std::size_t i = 0; i < tf.size(); ++i) {
    timer.start();
    /* int num_contact = */
    collide (o1, tf[i], o2, Id, request, results.rs[i]);
    timer.stop();
    results.times[i] = timer.getElapsedTimeInMicroSec();
  }
}

const char* sep = ", ";

void printResultHeaders ()
{
  std::cout << "Type 1" << sep << "Type 2" << sep << "mean time" << sep << "time std dev" << sep << "min time" << sep << "max time" << std::endl;
}

void printResults (const Geometry& g1, const Geometry& g2, const Results& rs)
{
  double mean = rs.times.mean();
  double var = rs.times.cwiseAbs2().mean() - mean*mean;
  std::cout << g1.type << sep << g2.type << sep << mean << sep << std::sqrt(var) << sep << rs.times.minCoeff() << sep << rs.times.maxCoeff() << std::endl;
}

// int main(int argc, char** argv)
int main(int, char**)
{
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  std::vector<Geometry> geoms;
// |            | box | sphere | capsule | cone | cylinder | plane | half-space | triangle |
  geoms.push_back(Geometry ("Box"       , new Box       (1, 2, 3)                                       ));
  geoms.push_back(Geometry ("Sphere"    , new Sphere    (2)                                             ));
  geoms.push_back(Geometry ("Capsule"   , new Capsule   (2, 1)                                          ));
  geoms.push_back(Geometry ("Cone"      , new Cone      (2, 1)                                          ));
  geoms.push_back(Geometry ("Cylinder"  , new Cylinder  (2, 1)                                          ));
  geoms.push_back(Geometry ("Plane"     , new Plane     (Vec3f(1,1,1).normalized(), 0)                  ));
  geoms.push_back(Geometry ("Halfspace" , new Halfspace (Vec3f(1,1,1).normalized(), 0)                  ));
  // not implemented // geoms.push_back(Geometry ("TriangleP" , new TriangleP (Vec3f(0,1,0), Vec3f(0,0,1), Vec3f(-1,0,0))     ));

  geoms.push_back(Geometry ("rob BVHModel<OBB>"   , objToGeom<OBB>   ((path / "rob.obj").string())));
  // geoms.push_back(Geometry ("rob BVHModel<RSS>"   , objToGeom<RSS>   ((path / "rob.obj").string())));
  // geoms.push_back(Geometry ("rob BVHModel<kIOS>"  , objToGeom<kIOS>  ((path / "rob.obj").string())));
  geoms.push_back(Geometry ("rob BVHModel<OBBRSS>", objToGeom<OBBRSS>((path / "rob.obj").string())));

  geoms.push_back(Geometry ("env BVHModel<OBB>"   , objToGeom<OBB>   ((path / "env.obj").string())));
  // geoms.push_back(Geometry ("env BVHModel<RSS>"   , objToGeom<RSS>   ((path / "env.obj").string())));
  // geoms.push_back(Geometry ("env BVHModel<kIOS>"  , objToGeom<kIOS>  ((path / "env.obj").string())));
  geoms.push_back(Geometry ("env BVHModel<OBBRSS>", objToGeom<OBBRSS>((path / "env.obj").string())));
  
  std::vector<Transform3f> transforms;
  FCL_REAL extents[] = {-20, -20, 0, 20, 20, 20};
  std::size_t n = 100;
  // bool verbose = false;

  generateRandomTransforms(extents, transforms, n);

  printResultHeaders();

  // collision
  CollisionRequest request;
  // request.num_max_contacts = 1;
  // request.enable_contact = false;
  // request.enable_distance_lower_bound = false;
  // request.num_max_cost_sources = 1;
  // request.enable_cost = false;
  // request.use_approximate_cost = true;
  // request.gjk_solver_type = GST_INDEP;
  for(std::size_t i = 0; i < geoms.size(); ++i) {
    for (std::size_t j = i; j < geoms.size(); ++j) {
      if (!supportedPair(geoms[i].o.get(), geoms[j].o.get())) continue;
      Results results (n);
      collide(transforms, geoms[i].o.get(), geoms[j].o.get(), request, results);
      printResults(geoms[i], geoms[j], results);
    }
  }

  // request.enable_distance_lower_bound = true;
  // for(std::size_t i = 0; i < geoms.size(); ++i)
  // {
    // Results results (n);
    // selfCollide(transforms, geoms[i].o.get(), request, results);
    // printResults(geoms[i], results);
  // }
}
