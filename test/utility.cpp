#include "utility.h"

#include <hpp/fcl/BV/BV.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shape_to_BVH_model.h>

#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>

#include <cstdio>
#include <cstddef>
#include <fstream>
#include <iostream>

namespace hpp {
namespace fcl {

BenchTimer::BenchTimer() {
#ifdef _WIN32
  QueryPerformanceFrequency(&frequency);
  startCount.QuadPart = 0;
  endCount.QuadPart = 0;
#else
  startCount.tv_sec = startCount.tv_usec = 0;
  endCount.tv_sec = endCount.tv_usec = 0;
#endif

  stopped = 0;
  startTimeInMicroSec = 0;
  endTimeInMicroSec = 0;
}

BenchTimer::~BenchTimer() {}

void BenchTimer::start() {
  stopped = 0;  // reset stop flag
#ifdef _WIN32
  QueryPerformanceCounter(&startCount);
#else
  gettimeofday(&startCount, NULL);
#endif
}

void BenchTimer::stop() {
  stopped = 1;  // set timer stopped flag

#ifdef _WIN32
  QueryPerformanceCounter(&endCount);
#else
  gettimeofday(&endCount, NULL);
#endif
}

double BenchTimer::getElapsedTimeInMicroSec() {
#ifdef _WIN32
  if (!stopped) QueryPerformanceCounter(&endCount);

  startTimeInMicroSec = startCount.QuadPart * (1000000.0 / frequency.QuadPart);
  endTimeInMicroSec = endCount.QuadPart * (1000000.0 / frequency.QuadPart);
#else
  if (!stopped) gettimeofday(&endCount, NULL);

  startTimeInMicroSec =
      ((double)startCount.tv_sec * 1000000.0) + (double)startCount.tv_usec;
  endTimeInMicroSec =
      ((double)endCount.tv_sec * 1000000.0) + (double)endCount.tv_usec;
#endif

  return endTimeInMicroSec - startTimeInMicroSec;
}

double BenchTimer::getElapsedTimeInMilliSec() {
  return this->getElapsedTimeInMicroSec() * 0.001;
}

double BenchTimer::getElapsedTimeInSec() {
  return this->getElapsedTimeInMicroSec() * 0.000001;
}

double BenchTimer::getElapsedTime() { return this->getElapsedTimeInMilliSec(); }

const Eigen::IOFormat vfmt = Eigen::IOFormat(
    Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
const Eigen::IOFormat pyfmt = Eigen::IOFormat(
    Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

const Vec3f UnitX = Vec3f(1, 0, 0);
const Vec3f UnitY = Vec3f(0, 1, 0);
const Vec3f UnitZ = Vec3f(0, 0, 1);

FCL_REAL rand_interval(FCL_REAL rmin, FCL_REAL rmax) {
  FCL_REAL t = rand() / ((FCL_REAL)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

void loadOBJFile(const char* filename, std::vector<Vec3f>& points,
                 std::vector<Triangle>& triangles) {
  FILE* file = fopen(filename, "rb");
  if (!file) {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while (fgets(line_buffer, 2000, file)) {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if (!first_token || first_token[0] == '#' || first_token[0] == 0) continue;

    switch (first_token[0]) {
      case 'v': {
        if (first_token[1] == 'n') {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        } else if (first_token[1] == 't') {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        } else {
          FCL_REAL x = (FCL_REAL)atof(strtok(NULL, "\t "));
          FCL_REAL y = (FCL_REAL)atof(strtok(NULL, "\t "));
          FCL_REAL z = (FCL_REAL)atof(strtok(NULL, "\t "));
          Vec3f p(x, y, z);
          points.push_back(p);
        }
      } break;
      case 'f': {
        Triangle tri;
        char* data[30];
        int n = 0;
        while ((data[n] = strtok(NULL, "\t \r\n")) != NULL) {
          if (strlen(data[n])) n++;
        }

        for (int t = 0; t < (n - 2); ++t) {
          if ((!has_texture) && (!has_normal)) {
            tri[0] = (Triangle::index_type)(atoi(data[0]) - 1);
            tri[1] = (Triangle::index_type)(atoi(data[1]) - 1);
            tri[2] = (Triangle::index_type)(atoi(data[2]) - 1);
          } else {
            const char* v1;
            for (Triangle::index_type i = 0; i < 3; i++) {
              // vertex ID
              if (i == 0)
                v1 = data[0];
              else
                v1 = data[(Triangle::index_type)t + i];

              tri[i] = (Triangle::index_type)(atoi(v1) - 1);
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}

void saveOBJFile(const char* filename, std::vector<Vec3f>& points,
                 std::vector<Triangle>& triangles) {
  std::ofstream os(filename);
  if (!os) {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  for (std::size_t i = 0; i < points.size(); ++i) {
    os << "v " << points[i][0] << " " << points[i][1] << " " << points[i][2]
       << std::endl;
  }

  for (std::size_t i = 0; i < triangles.size(); ++i) {
    os << "f " << triangles[i][0] + 1 << " " << triangles[i][1] + 1 << " "
       << triangles[i][2] + 1 << std::endl;
  }

  os.close();
}

#ifdef HPP_FCL_HAS_OCTOMAP
OcTree loadOctreeFile(const std::string& filename, const FCL_REAL& resolution) {
  octomap::OcTreePtr_t octree(new octomap::OcTree(filename));
  if (octree->getResolution() != resolution) {
    std::ostringstream oss;
    oss << "Resolution of the OcTree is " << octree->getResolution()
        << " and not " << resolution;
    throw std::invalid_argument(oss.str());
  }
  return hpp::fcl::OcTree(octree);
}
#endif

void eulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Matrix3f& R) {
  FCL_REAL c1 = cos(a);
  FCL_REAL c2 = cos(b);
  FCL_REAL c3 = cos(c);
  FCL_REAL s1 = sin(a);
  FCL_REAL s2 = sin(b);
  FCL_REAL s3 = sin(c);

  R << c1 * c2, -c2 * s1, s2, c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3,
      -c2 * s3, s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3;
}

void generateRandomTransform(FCL_REAL extents[6], Transform3f& transform) {
  FCL_REAL x = rand_interval(extents[0], extents[3]);
  FCL_REAL y = rand_interval(extents[1], extents[4]);
  FCL_REAL z = rand_interval(extents[2], extents[5]);

  const FCL_REAL pi = 3.1415926;
  FCL_REAL a = rand_interval(0, 2 * pi);
  FCL_REAL b = rand_interval(0, 2 * pi);
  FCL_REAL c = rand_interval(0, 2 * pi);

  Matrix3f R;
  eulerToMatrix(a, b, c, R);
  Vec3f T(x, y, z);
  transform.setTransform(R, T);
}

void generateRandomTransforms(FCL_REAL extents[6],
                              std::vector<Transform3f>& transforms,
                              std::size_t n) {
  transforms.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    FCL_REAL x = rand_interval(extents[0], extents[3]);
    FCL_REAL y = rand_interval(extents[1], extents[4]);
    FCL_REAL z = rand_interval(extents[2], extents[5]);

    const FCL_REAL pi = 3.1415926;
    FCL_REAL a = rand_interval(0, 2 * pi);
    FCL_REAL b = rand_interval(0, 2 * pi);
    FCL_REAL c = rand_interval(0, 2 * pi);

    {
      Matrix3f R;
      eulerToMatrix(a, b, c, R);
      Vec3f T(x, y, z);
      transforms[i].setTransform(R, T);
    }
  }
}

void generateRandomTransforms(FCL_REAL extents[6], FCL_REAL delta_trans[3],
                              FCL_REAL delta_rot,
                              std::vector<Transform3f>& transforms,
                              std::vector<Transform3f>& transforms2,
                              std::size_t n) {
  transforms.resize(n);
  transforms2.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    FCL_REAL x = rand_interval(extents[0], extents[3]);
    FCL_REAL y = rand_interval(extents[1], extents[4]);
    FCL_REAL z = rand_interval(extents[2], extents[5]);

    const FCL_REAL pi = 3.1415926;
    FCL_REAL a = rand_interval(0, 2 * pi);
    FCL_REAL b = rand_interval(0, 2 * pi);
    FCL_REAL c = rand_interval(0, 2 * pi);

    {
      Matrix3f R;
      eulerToMatrix(a, b, c, R);
      Vec3f T(x, y, z);
      transforms[i].setTransform(R, T);
    }

    FCL_REAL deltax = rand_interval(-delta_trans[0], delta_trans[0]);
    FCL_REAL deltay = rand_interval(-delta_trans[1], delta_trans[1]);
    FCL_REAL deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

    FCL_REAL deltaa = rand_interval(-delta_rot, delta_rot);
    FCL_REAL deltab = rand_interval(-delta_rot, delta_rot);
    FCL_REAL deltac = rand_interval(-delta_rot, delta_rot);

    {
      Matrix3f R;
      eulerToMatrix(a + deltaa, b + deltab, c + deltac, R);
      Vec3f T(x + deltax, y + deltay, z + deltaz);
      transforms2[i].setTransform(R, T);
    }
  }
}

bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2,
                              void* cdata_) {
  CollisionData* cdata = static_cast<CollisionData*>(cdata_);
  const CollisionRequest& request = cdata->request;
  CollisionResult& result = cdata->result;

  if (cdata->done) return true;

  collide(o1, o2, request, result);

  if ((result.isCollision()) &&
      (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2,
                             void* cdata_, FCL_REAL& dist) {
  DistanceData* cdata = static_cast<DistanceData*>(cdata_);
  const DistanceRequest& request = cdata->request;
  DistanceResult& result = cdata->result;

  if (cdata->done) {
    dist = result.min_distance;
    return true;
  }

  distance(o1, o2, request, result);

  dist = result.min_distance;

  if (dist <= 0) return true;  // in collision or in touch

  return cdata->done;
}

std::string getNodeTypeName(NODE_TYPE node_type) {
  if (node_type == BV_UNKNOWN)
    return std::string("BV_UNKNOWN");
  else if (node_type == BV_AABB)
    return std::string("BV_AABB");
  else if (node_type == BV_OBB)
    return std::string("BV_OBB");
  else if (node_type == BV_RSS)
    return std::string("BV_RSS");
  else if (node_type == BV_kIOS)
    return std::string("BV_kIOS");
  else if (node_type == BV_OBBRSS)
    return std::string("BV_OBBRSS");
  else if (node_type == BV_KDOP16)
    return std::string("BV_KDOP16");
  else if (node_type == BV_KDOP18)
    return std::string("BV_KDOP18");
  else if (node_type == BV_KDOP24)
    return std::string("BV_KDOP24");
  else if (node_type == GEOM_BOX)
    return std::string("GEOM_BOX");
  else if (node_type == GEOM_SPHERE)
    return std::string("GEOM_SPHERE");
  else if (node_type == GEOM_CAPSULE)
    return std::string("GEOM_CAPSULE");
  else if (node_type == GEOM_CONE)
    return std::string("GEOM_CONE");
  else if (node_type == GEOM_CYLINDER)
    return std::string("GEOM_CYLINDER");
  else if (node_type == GEOM_CONVEX)
    return std::string("GEOM_CONVEX");
  else if (node_type == GEOM_PLANE)
    return std::string("GEOM_PLANE");
  else if (node_type == GEOM_HALFSPACE)
    return std::string("GEOM_HALFSPACE");
  else if (node_type == GEOM_TRIANGLE)
    return std::string("GEOM_TRIANGLE");
  else if (node_type == GEOM_OCTREE)
    return std::string("GEOM_OCTREE");
  else
    return std::string("invalid");
}

Quaternion3f makeQuat(FCL_REAL w, FCL_REAL x, FCL_REAL y, FCL_REAL z) {
  Quaternion3f q;
  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  return q;
}

std::ostream& operator<<(std::ostream& os, const Transform3f& tf) {
  return os << "[ " << tf.getTranslation().format(vfmt) << ", "
            << tf.getQuatRotation().coeffs().format(vfmt) << " ]";
}

std::size_t getNbRun(const int& argc, char const* const* argv,
                     std::size_t defaultValue) {
  for (int i = 0; i < argc; ++i)
    if (strcmp(argv[i], "--nb-run") == 0)
      if (i + 1 != argc) return (std::size_t)strtol(argv[i + 1], NULL, 10);
  return defaultValue;
}

void generateEnvironments(std::vector<CollisionObject*>& env,
                          FCL_REAL env_scale, std::size_t n) {
  FCL_REAL extents[] = {-env_scale, env_scale,  -env_scale,
                        env_scale,  -env_scale, env_scale};
  std::vector<Transform3f> transforms(n);

  generateRandomTransforms(extents, transforms, n);
  for (std::size_t i = 0; i < n; ++i) {
    Box* box = new Box(5, 10, 20);
    env.push_back(
        new CollisionObject(shared_ptr<CollisionGeometry>(box), transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  generateRandomTransforms(extents, transforms, n);
  for (std::size_t i = 0; i < n; ++i) {
    Sphere* sphere = new Sphere(30);
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(sphere),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  generateRandomTransforms(extents, transforms, n);
  for (std::size_t i = 0; i < n; ++i) {
    Cylinder* cylinder = new Cylinder(10, 40);
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(cylinder),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }
}

void generateEnvironmentsMesh(std::vector<CollisionObject*>& env,
                              FCL_REAL env_scale, std::size_t n) {
  FCL_REAL extents[] = {-env_scale, env_scale,  -env_scale,
                        env_scale,  -env_scale, env_scale};
  std::vector<Transform3f> transforms;

  generateRandomTransforms(extents, transforms, n);
  Box box(5, 10, 20);
  for (std::size_t i = 0; i < n; ++i) {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, box, Transform3f::Identity());
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(model),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  generateRandomTransforms(extents, transforms, n);
  Sphere sphere(30);
  for (std::size_t i = 0; i < n; ++i) {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, sphere, Transform3f::Identity(), 16, 16);
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(model),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }

  generateRandomTransforms(extents, transforms, n);
  Cylinder cylinder(10, 40);
  for (std::size_t i = 0; i < n; ++i) {
    BVHModel<OBBRSS>* model = new BVHModel<OBBRSS>();
    generateBVHModel(*model, cylinder, Transform3f::Identity(), 16, 16);
    env.push_back(new CollisionObject(shared_ptr<CollisionGeometry>(model),
                                      transforms[i]));
    env.back()->collisionGeometry()->computeLocalAABB();
  }
}

/// Takes a point and projects it onto the surface of the unit sphere
void toSphere(Vec3f& point) {
  assert(point.norm() > 1e-8);
  point /= point.norm();
}

/// Takes a point, projects it on the unit sphere and applies an ellipsoid
/// transformation to it. A point x belongs to the surface of an ellipsoid if
/// x^T * A * x = 1, where A = diag(1/r**2) with r being the radii of the
/// ellipsoid. Thus, the point y = A^(1/2) * x belongs to the unit sphere if y *
/// y = 1. Therefore, the tranformation which brings y to x is A^(-1/2) =
/// diag(r).
void toEllipsoid(Vec3f& point, const Ellipsoid& ellipsoid) {
  toSphere(point);
  point[0] *= ellipsoid.radii[0];
  point[1] *= ellipsoid.radii[1];
  point[2] *= ellipsoid.radii[2];
}

Convex<Triangle> constructPolytopeFromEllipsoid(const Ellipsoid& ellipsoid) {
  FCL_REAL PHI = (1 + std::sqrt(5)) / 2;

  // vertices
  Vec3f* pts = new Vec3f[12];
  pts[0] = Vec3f(-1, PHI, 0);
  pts[1] = Vec3f(1, PHI, 0);
  pts[2] = Vec3f(-1, -PHI, 0);
  pts[3] = Vec3f(1, -PHI, 0);

  pts[4] = Vec3f(0, -1, PHI);
  pts[5] = Vec3f(0, 1, PHI);
  pts[6] = Vec3f(0, -1, -PHI);
  pts[7] = Vec3f(0, 1, -PHI);

  pts[8] = Vec3f(PHI, 0, -1);
  pts[9] = Vec3f(PHI, 0, 1);
  pts[10] = Vec3f(-PHI, 0, -1);
  pts[11] = Vec3f(-PHI, 0, 1);

  for (int i = 0; i < 12; ++i) {
    toEllipsoid(pts[i], ellipsoid);
  }

  // faces
  Triangle* tris = new Triangle[20];
  tris[0].set(0, 11, 5);
  tris[1].set(0, 5, 1);
  tris[2].set(0, 1, 7);
  tris[3].set(0, 7, 10);
  tris[4].set(0, 10, 11);

  tris[5].set(1, 5, 9);
  tris[6].set(5, 11, 4);
  tris[7].set(11, 10, 2);
  tris[8].set(10, 7, 6);
  tris[9].set(7, 1, 8);

  tris[10].set(3, 9, 4);
  tris[11].set(3, 4, 2);
  tris[12].set(3, 2, 6);
  tris[13].set(3, 6, 8);
  tris[14].set(3, 8, 9);

  tris[15].set(4, 9, 5);
  tris[16].set(2, 4, 11);
  tris[17].set(6, 2, 10);
  tris[18].set(8, 6, 7);
  tris[19].set(9, 8, 1);
  return Convex<Triangle>(true,
                          pts,   // points
                          12,    // num_points
                          tris,  // triangles
                          20     // number of triangles
  );
}

}  // namespace fcl

}  // namespace hpp
