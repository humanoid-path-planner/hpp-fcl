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
#include <hpp/fcl/collision_utility.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/collision_func_matrix.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include "utility.h"
#include "fcl_resources/config.h"

using namespace hpp::fcl;

CollisionFunctionMatrix lookupTable;
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

template <typename BV/*, SplitMethodType split_method*/>
CollisionGeometryPtr_t meshToGeom (const std::string& filename, const Vec3f& scale = Vec3f(1, 1, 1))
{
  boost::shared_ptr< BVHModel<BV> > model (new BVHModel<BV>());
  loadPolyhedronFromResource(filename, scale, model);
  return model;
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

#ifndef NDEBUG // if debug mode
int Ntransform = 1;
#else
int Ntransform = 100;
#endif
FCL_REAL limit = 20;
bool verbose = false;

#define OUT(x) if (verbose) std::cout << x << std::endl
#define CHECK_PARAM_NB(NB, NAME) \
    if (iarg + NB >= argc) throw std::invalid_argument(#NAME " requires " #NB " numbers")
void handleParam (int& iarg, const int& argc, char** argv, CollisionRequest& request)
{
  while (iarg < argc) {
    std::string a(argv[iarg]);
    if (a == "-nb_transform") {
      CHECK_PARAM_NB(1, nb_transform);
      Ntransform = atoi(argv[iarg+1]);
      OUT("nb_transform = " << Ntransform);
      iarg += 2;
    } else if (a == "-enable_distance_lower_bound") {
      CHECK_PARAM_NB(1, enable_distance_lower_bound);
      request.enable_distance_lower_bound = bool(atoi(argv[iarg+1]));
      iarg += 2;
    } else if (a == "-limit") {
      CHECK_PARAM_NB(1, limit);
      limit = atof(argv[iarg+1]);
      iarg += 2;
    } else if (a == "-verbose") {
      verbose = true;
      iarg += 1;
    } else {
      break;
    }
  }
}
#define CREATE_SHAPE_2(var, Name) \
    CHECK_PARAM_NB(2, Name); \
    var.reset(new Name (atof(argv[iarg+1]), atof(argv[iarg+2]))); \
     iarg += 3;
Geometry makeGeomFromParam(int& iarg, const int& argc, char** argv)
{
  if (iarg >= argc) throw std::invalid_argument("An argument is required.");
  std::string a(argv[iarg]);
  std::string type;
  CollisionGeometryPtr_t o;
  if (a == "-box") {
    CHECK_PARAM_NB(3, Box);
    o.reset(new Box (atof(argv[iarg+1]),
                     atof(argv[iarg+2]),
                     atof(argv[iarg+3])));
    iarg += 4;
    type = "box";
  } else if (a == "-sphere") {
    CHECK_PARAM_NB(1, Sphere);
    o.reset(new Sphere (atof(argv[iarg+1])));
    iarg += 2;
    type = "sphere";
  } else if (a == "-mesh") {
    CHECK_PARAM_NB(2, Mesh);
    OUT("Loading " << argv[iarg+2] << " as BVHModel<" << argv[iarg+1] << ">...");
    if (strcmp(argv[iarg+1], "obb") == 0) {
      o = meshToGeom<OBB>(argv[iarg+2]);
      OUT("Mesh has " << boost::dynamic_pointer_cast<BVHModel<OBB> >(o)->num_tris << " triangles");
      type = "mesh_obb";
    }
    else if (strcmp(argv[iarg+1], "obbrss") == 0) {
      o = meshToGeom<OBBRSS>(argv[iarg+2]);
      OUT("Mesh has " << boost::dynamic_pointer_cast<BVHModel<OBBRSS> >(o)->num_tris << " triangles");
      type = "mesh_obbrss";
    }
    else
      throw std::invalid_argument ("BV type must be obb or obbrss");
    OUT("done.");
    iarg += 3;
    if (iarg < argc && strcmp(argv[iarg], "crop") == 0) {
      CHECK_PARAM_NB(6, Crop);
      hpp::fcl::AABB aabb(
          Vec3f(atof(argv[iarg+1]),
                atof(argv[iarg+2]),
                atof(argv[iarg+3])),
          Vec3f(atof(argv[iarg+4]),
                atof(argv[iarg+5]),
                atof(argv[iarg+6])));
      OUT("Cropping " << aabb.min_.transpose() << " ---- " << aabb.max_.transpose() << " ...");
      o->computeLocalAABB();
      OUT("Mesh AABB is " << o->aabb_local.min_.transpose() << " ---- " << o->aabb_local.max_.transpose() << " ...");
      o.reset(extract(o.get(), Transform3f(), aabb));
      if (!o) throw std::invalid_argument ("Failed to crop.");
      OUT("Crop has " << boost::dynamic_pointer_cast<BVHModel<OBB> >(o)->num_tris << " triangles");
      iarg += 7;
    }
  } else if (a == "-capsule") {
    CREATE_SHAPE_2(o, Capsule);
    type = "capsule";
  } else if (a == "-cone") {
    CREATE_SHAPE_2(o, Cone);
    type = "cone";
  } else if (a == "-cylinder") {
    CREATE_SHAPE_2(o, Cylinder);
    type = "cylinder";
  } else {
    throw std::invalid_argument (std::string("Unknown argument: ") + a);
  }
  return Geometry(type, o);
}

int main(int argc, char** argv)
{
  
  std::vector<Transform3f> transforms;

  CollisionRequest request;

  if (argc > 1) {
    int iarg = 1;
    handleParam(iarg, argc, argv, request);
    Geometry first  = makeGeomFromParam(iarg, argc, argv);
    Geometry second = makeGeomFromParam(iarg, argc, argv);

    FCL_REAL extents[] = {-limit, -limit, -limit, limit, limit, limit};
    generateRandomTransforms(extents, transforms, Ntransform);
    printResultHeaders();
    Results results (Ntransform);
    collide(transforms, first.o.get(), second.o.get(), request, results);
    printResults(first, second, results);
  } else {
    FCL_REAL extents[] = {-limit, -limit, -limit, limit, limit, limit};
    generateRandomTransforms(extents, transforms, Ntransform);
    boost::filesystem::path path(TEST_RESOURCES_DIR);

    std::vector<Geometry> geoms;
    geoms.push_back(Geometry ("Box"       , new Box       (1, 2, 3)                                       ));
    geoms.push_back(Geometry ("Sphere"    , new Sphere    (2)                                             ));
    geoms.push_back(Geometry ("Capsule"   , new Capsule   (2, 1)                                          ));
    geoms.push_back(Geometry ("Cone"      , new Cone      (2, 1)                                          ));
    geoms.push_back(Geometry ("Cylinder"  , new Cylinder  (2, 1)                                          ));
    //geoms.push_back(Geometry ("Plane"     , new Plane     (Vec3f(1,1,1).normalized(), 0)                  ));
    //geoms.push_back(Geometry ("Halfspace" , new Halfspace (Vec3f(1,1,1).normalized(), 0)                  ));
    // not implemented // geoms.push_back(Geometry ("TriangleP" , new TriangleP (Vec3f(0,1,0), Vec3f(0,0,1), Vec3f(-1,0,0))     ));

    geoms.push_back(Geometry ("rob BVHModel<OBB>"   , objToGeom<OBB>   ((path / "rob.obj").string())));
    // geoms.push_back(Geometry ("rob BVHModel<RSS>"   , objToGeom<RSS>   ((path / "rob.obj").string())));
    // geoms.push_back(Geometry ("rob BVHModel<kIOS>"  , objToGeom<kIOS>  ((path / "rob.obj").string())));
    geoms.push_back(Geometry ("rob BVHModel<OBBRSS>", objToGeom<OBBRSS>((path / "rob.obj").string())));

    geoms.push_back(Geometry ("env BVHModel<OBB>"   , objToGeom<OBB>   ((path / "env.obj").string())));
    // geoms.push_back(Geometry ("env BVHModel<RSS>"   , objToGeom<RSS>   ((path / "env.obj").string())));
    // geoms.push_back(Geometry ("env BVHModel<kIOS>"  , objToGeom<kIOS>  ((path / "env.obj").string())));
    geoms.push_back(Geometry ("env BVHModel<OBBRSS>", objToGeom<OBBRSS>((path / "env.obj").string())));

    printResultHeaders();

    // collision
    for(std::size_t i = 0; i < geoms.size(); ++i) {
      for (std::size_t j = i; j < geoms.size(); ++j) {
        if (!supportedPair(geoms[i].o.get(), geoms[j].o.get())) continue;
        Results results (Ntransform);
        collide(transforms, geoms[i].o.get(), geoms[j].o.get(), request, results);
        printResults(geoms[i], geoms[j], results);
      }
    }
  }

  return 0;
}
