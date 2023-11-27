# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

## [2.4.0] - 2023-11-27

### Added
- Add method to `CollisionObject` to get `CollisionGeometry` raw pointer

### Fixed
- Fix RPATH computation on OSX
- Fix Python stubs generation on Windows

## [2.3.7] - 2023-11-15

### What's Changed
- Add Python 3.12 support by @jorisv in https://github.com/humanoid-path-planner/hpp-fcl/pull/471
- Enable ruff linting by @nim65s https://github.com/humanoid-path-planner/hpp-fcl/pull/464

## [2.3.6] - 2023-09-30

### What's Changed
- Update ROS_DISTRO by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/442
- Add citations by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/449
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/444
- [WIP] Debug by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/455
- CMake: require >= 3.10 by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/453
- core: fix SaPCollisionManager::empty() by @rujialiu in https://github.com/humanoid-path-planner/hpp-fcl/pull/454
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/452

### New Contributors
- @rujialiu made their first contribution in https://github.com/humanoid-path-planner/hpp-fcl/pull/454

## [2.3.5] - 2023-07-11

### What's Changed
- Fix compilation warning by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/434
- Fix parsing of doxygen doc by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/439
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/438

## [2.3.4] - 2023-06-01

### What's Changed
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/414
- Fix conversion warning by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/417
- Add missing boost include by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/418
- ci: update macos-linux-pip by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/419
- Modernize Cmake use by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/420
- tests: use boost::filesystem by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/424
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/425
- Update minimal Python version  by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/427
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/430
- Sync submodule CMake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/431

## [2.3.3] - 2023-05-09

### What's Changed
- update default C++ to 14 by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/410
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/413

## [2.3.2] - 2023-04-27

### What's Changed
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/391
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/393
- Topic/rpath by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/394
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/396
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/399
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/402
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/406

## [2.3.1] - 2023-03-25

### What's Changed
- Remove useless call to /proc/cpuinfo by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/385
- Add pip CI by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/386
- [GJKSolver] Fix missing switch case in result status of GJK by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/387
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/388

## [2.3.0] - 2023-03-17

### What's Changed
- [CI] Remove EOL Galactic by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/366
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/367
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/368
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/369
- Adding EarlyStopped flag in GJK by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/371
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/373
- Update CI by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/374
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/375
- Skip test if BUILD_TESTING is OFF by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/378

## [2.2.0] - 2022-12-12

### What's Changed
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/358
- Extract checks if AABB overlap by @jmirabel in https://github.com/humanoid-path-planner/hpp-fcl/pull/360
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/361
- Sync submodule CMake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/362
- Add support of Pickling by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/363

## [2.1.4] - 2022-10-24

### What's Changed
- Sync submodule CMake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/352
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/353

## [2.1.3] - 2022-09-13

### What's Changed
- Minor boost cleanup by @pantor in https://github.com/humanoid-path-planner/hpp-fcl/pull/331
- [CI] Activate ROS2 configurations by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/332
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/337
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/341
- Fix shapeIntersect when for EPA FallBack by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/342
- Fix findAssimp on Windows by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/345
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/347

### New Contributors
- @pantor made their first contribution in https://github.com/humanoid-path-planner/hpp-fcl/pull/331

## [2.1.2] - 2022-08-01

### What's Changed
- core: add EPA::FallBack condition to shapeDistance computation by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/325
- CMake: update to eigenpy 2.7.10 by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/327

## [2.1.1] - 2022-07-25

### What's Changed
- cmake: relocatable package for recent CMake versions by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/319
- ROS2/Colcon integration by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/321

## [2.1.0] - 2022-07-13

### What's Changed
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/288
- Add enum helpers by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/290
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/294
- Ellipsoids in collision & distance matrices by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/295
- doc: simplex projection in GJK class. by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/296
- Feature: Nesterov acceleration for GJK by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/289
- Add more testing to broadphase by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/298
- Feature: adding convergence criterions for GJK algorithm by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/299
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/300
- Reorder triangles when computing convex hulls by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/301
- Exposing gjk utils by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/302
- Fix assert precision in GJK by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/304
- Simplify GJKSolver settings by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/305
- Add CollisionResult::nearest_points  by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/303
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/306
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/309
- Fix minimal value for GJK::distance_upper_bound by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/310
- Fix incoherent overlap by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/311
- Expose shared_ptr<OcTree> by @Jiayuan-Gu in https://github.com/humanoid-path-planner/hpp-fcl/pull/314
- test/gjk_convergence_criterion: Add check on GJK::Status by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/315
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/316
- Handle negative security margin by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/312

### New Contributors
- @Jiayuan-Gu made their first contribution in https://github.com/humanoid-path-planner/hpp-fcl/pull/314

## [2.0.1] - 2022-04-15

This PR mainly fixes packaging issues and removes compilation warnings.

### What's Changed
- Zero warnings by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/282
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/283
- [pre-commit.ci] pre-commit autoupdate by @pre-commit-ci in https://github.com/humanoid-path-planner/hpp-fcl/pull/284
- Activate python3-pylatexenc dependency by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/286
- Comment pylatexenc again since it's not available on the buildfarm by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/287

### New Contributors
- @pre-commit-ci made their first contribution in https://github.com/humanoid-path-planner/hpp-fcl/pull/284

## [2.0.0] - 2022-04-06

This new release reintroduces the full support of Broad phase within hpp-fcl while also enforcing C++11 as minimal standard.

### What's Changed
- Add Ellipsoid by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/259
- Removing comment about inflation. by @lmontaut in https://github.com/humanoid-path-planner/hpp-fcl/pull/261
- Reintroduce broadphase by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/260
- Simplify CollisionObject by removing cgeom_const by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/263
- Address some warnings by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/262
- Fix missing copy of aabb_local in CollisionGeometry by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/264
- use std::shared_ptr, fix #218 by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/266
- Fix broadphase warnings for clang (some conversion remain for g++) by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/268
- [ComputeCollision] Return no collision if security_margin is set to -inf by @florent-lamiraux in https://github.com/humanoid-path-planner/hpp-fcl/pull/271
- tests: remove link to boost unit test framework by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/270
- Fix computation of aabb_center by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/273
- Add operator== and operator!= to CollisionGeometry by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/274
- Merge pull request #276 from humanoid-path-planner/patch-release-1.8.1 by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/277
- Fix some missing features in base classes by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/275
- Add operator{==,!=} to CollisionObject by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/278
- Configure and apply pre-commit by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/280
- Fix DistanceCallBackBaseWrapper by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/281

### New Contributors
- @lmontaut made their first contribution in https://github.com/humanoid-path-planner/hpp-fcl/pull/261

## [1.8.1] - 2022-03-20

### What's Changed
- Preparing for ROS1 and ROS2 release by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/255
- Patch release 1.8.1 by @wxmerkt in https://github.com/humanoid-path-planner/hpp-fcl/pull/276

## [1.8.0] - 2022-02-08

### What's Changed
- [CMake] Qhull is a private dependency by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/247
- Remove useless warnings by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/248
- fix submodule url by @nim65s in https://github.com/humanoid-path-planner/hpp-fcl/pull/246
- Remove warnings and add missing noalias by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/249
- Function makeOctree returns a shared pointer by @florent-lamiraux in https://github.com/humanoid-path-planner/hpp-fcl/pull/254
- Add support of HeightField by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/251
- [OcTree] Add method to save octree in obj file. by @florent-lamiraux in https://github.com/humanoid-path-planner/hpp-fcl/pull/256
- Fix C++98 compatibility by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/258

## [1.7.8] - 2021-10-30

### What's Changed
- Fix conversion by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/242
- Fix exposition of vertices by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/243
- Enhance Convex exposition by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/244
- Sync submodule cmake by @jcarpent in https://github.com/humanoid-path-planner/hpp-fcl/pull/245

## [1.7.7] - 2021-09-13

This new release fixes several bugs within the framework.

## [1.7.6] - 2021-09-08

This new release improves the packaging of the project and integrates the Stub generation of Python bindings.

## [1.7.5] - 2021-07-30

This new release provides extended API exposition in Python, removes some code related to CDD while also trying to rely on the QHULL version present on the system.

## [1.7.4] - 2021-06-11

This release fixes several bugs:
- correct update of the distance lower bound
- fix memory footprint computation

while also removing the support of Travis CI.

## [1.7.3] - 2021-05-26

This new release provides:
- fixes of LINE and POINTS when loading meshes with assimp
- removing of various warnings
- computation of memory footprint for geometries

## [1.7.2] - 2021-04-19

This new release improves the loading of meshes using Assimp by automatically removing degenerated LINES and POINTS.

## [1.7.1] - 2021-04-02

This new release reduces the impact of timers on the computations.
This should be used with care and can be enabled by setting the correct flag to true in the QueryRequest.

## [1.7.0] - 2021-03-31

This new release provides:
- extended support for serialization
- timing of the collision/distance computations
- helpers to build octree
- various bug fixes and interface improvements

## [1.6.0] - 2020-10-06

This new release provides:
- functors for evaluating Collision and Distances (faster call)
- extended support of v142 compiler
- support of collision check between HalfSpace and Convex shapes
- improvement of GJK solver
- fixes on Python bindings

## [1.5.4] - 2020-09-22

In this new release, the support of collision checking between Convex objects and HalfSpace have been enhanced and some minor fixes have been provided.

## [1.5.3] - 2020-08-31

This new release provides better CMake packaging and improved GJK algorithms.

## [1.5.2] - 2020-08-15

This release improves the packaging of the project and provides fixes for the GJK solver.

## [1.5.1] - 2020-08-06

This new release fixes packaging issues with precedent release 1.5.0. It also provides additional fixes in main collision/distance algorithms.

## [1.4.6] - 2020-06-10

This new release enhances the packaging of the project and allows the compilation of FCL on Windows systems.

## [1.4.5] - 2020-06-03

Changes in v1.4.5:
- Fix Python 3 doc generation
- Fix packaging of the project
- Compilation on Windows.
- [CMake] Install missing header.
- Add collide and distance prototype that update the GJK guess.
- Add support function cached guess in queries and merge query attribute.
- Add function to generate the convex hull.
- Add hint to the support function + Fix usage of GJK guess.
- [Python] Add constructor for class Convex.
- [Python] Bind functions to create BVHModel.

## [1.4.4] - 2020-04-29

Changes in 1.4.4:
- add MeshLoader::loadOctree
- fix generation of XML documentation
- fix generation of Doxygen documentation

## [1.4.3] - 2020-04-08

This new release fixes some packagings issues for OS X systems.

## [1.4.2] - 2020-04-04

Changes in v1.4.2:
- don't require linking to eigenpy in .pc file.

## [1.4.1] - 2020-04-03

Changes in v1.4.1:
- Bug fix + prepare optimization of collision using GJK / EPA
- Add missing constructor for Transform3f

## [1.4.0] - 2020-04-03

Changes since v1.3.0:
- Improve code efficiency + use shared memory between Numpy and Eigen
- [Python] Doc and minor update + [C++] bugfix
- [Python] Fix bindings of CollisionResult.
- FIX: throw when no contact is available
- Minor fix and computational improvments
- [GJK/EPA] Fix bugs + Treat sphere as point and capsule as line segment.
- Fix boxSphereDistance
- Provide documentation for the Python bindings.
- Generate Python documentation from doxygen documentation.
- Fix issue when Python_EXECUTABLE is not defined
- update CMake packaging

## [1.3.0] - 2020-01-28

This new release comes with:
- the removing of the GJK solver
- the Python bindings build by default
- an improved documentation
- additional Python bindings

## [1.2.2] - 2019-12-17

This new Release improves the Python bindings and fixes an important bug when checking the collision between two Capsules.

Thanks to @rstrudel for this fix.

## [1.2.1] - 2019-12-09

This new release improves both the packaging of the project, which seems to be totally compatible with the new CMake linkage style. In addition, the bindings are now fully compatible with Pinocchio.

## [1.2.0] - 2019-11-22

Changes since v1.1.3:
- Add python bindings
- Update CMake
- Add version support
- New folder Internal for internal header
- Travis: update CI & change policy to only perform build in DEBUG mode on Bionic
- assimp: fix issue with recent version of assimp
- [bindings] [CMakeLists] Use .so for Mac and .pyd for Windows, fix #86
- Organize documentation
- [CMake] fix octomap detection
- [Minor] update CMake module + fix visibility of some methods.
- Enable Convex / Convex queries + Add Python bindings.
- Fix unit-tests and compilation
- [GJK] Fix GJK::encloseOrigin (fixes unit-tests)
- Improve GJK implementation + OBB overlap test + bug fixes
- Clean include  BV/BVH/math/mesh_loader

## [1.1.3] - 2019-08-07

This new release enhances the compatibility of hpp-fcl with C++14 and more.
This feature is requested for integration in Anaconda.

## [1.1.2] - 2019-08-05

This new release provides a fix in the parallelization of the computations and improves the packaging of the whole project.

## [1.0.2] - 2019-04-24

Changes since v1.0.1:
- obb: fix compatibility with Eigen 3.0.5
- [CI] octomap for osx

## [1.0.1] - 2019-02-20

- Fix CI on OSX
- Declare CachedMeshLoader::Key::operator<
- minor details

## [0.7.0] - 2019-01-31

This release is mainly here to allow the packaging of HPP-RBPRM.
Another release will follow with more news.

## [0.6.0] - 2018-10-22

- Fix bug when OCTOMAP is not found
- move buildTrianglePlane and clipTriangle method from private to public
- Fix bug with "\" symbols
- [CMake] Add flags related to Octomap in pkg-config file and remove FCL_HAVE_EIGEN

## [0.5.1] - 2017-10-02

Now Eigen is at the heart of linear algebra computations.

## [0.5] - 2017-03-17

First release


[Unreleased]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.4.0...HEAD
[2.4.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.3.7...v2.4.0
[2.3.7]: https://github.com/humanoid-path-planner/hpp-fcl/compare/2.3.6...v2.3.7
[2.3.6]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.3.5...v2.3.6
[2.3.5]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.3.4...v2.3.5
[2.3.4]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.3.3...v2.3.4
[2.3.3]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.3.2...v2.3.3
[2.3.2]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.3.1...v2.3.2
[2.3.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.3.0...v2.3.1
[2.3.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.2.0...v2.3.0
[2.2.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.1.4...v2.2.0
[2.1.4]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.1.3...v2.1.4
[2.1.3]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.1.2...v2.1.3
[2.1.2]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.1.1...v2.1.2
[2.1.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.1.0...v2.1.1
[2.1.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.0.1...v2.1.0
[2.0.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v2.0.0...v2.0.1
[2.0.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.8.1...v2.0.0
[1.8.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.8.0...v1.8.1
[1.8.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.7.8...v1.8.0
[1.7.8]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.7.7...v1.7.8
[1.7.7]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.7.6...v1.7.7
[1.7.6]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.7.5...v1.7.6
[1.7.5]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.7.4...v1.7.5
[1.7.4]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.7.3...v1.7.4
[1.7.3]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.7.2...v1.7.3
[1.7.2]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.7.1...v1.7.2
[1.7.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.7.0...v1.7.1
[1.7.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.6.0...v1.7.0
[1.6.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.5.4...v1.6.0
[1.5.4]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.5.3...v1.5.4
[1.5.3]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.5.2...v1.5.3
[1.5.2]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.5.1...v1.5.2
[1.5.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.4.6...v1.5.1
[1.4.6]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.4.5...v1.4.6
[1.4.5]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.4.4...v1.4.5
[1.4.4]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.4.3...v1.4.4
[1.4.3]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.4.2...v1.4.3
[1.4.2]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.4.1...v1.4.2
[1.4.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.4.0...v1.4.1
[1.4.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.3.0...v1.4.0
[1.3.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.2.2...v1.3.0
[1.2.2]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.2.1...v1.2.2
[1.2.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.1.3...v1.2.0
[1.1.3]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.1.2...v1.1.3
[1.1.2]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.0.2...v1.1.2
[1.0.2]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v1.0.1...v1.0.2
[1.0.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v0.7.0...v1.0.1
[0.7.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v0.5.1...v0.6.0
[0.5.1]: https://github.com/humanoid-path-planner/hpp-fcl/compare/v0.5...v0.5.1
[0.5]: https://github.com/humanoid-path-planner/hpp-fcl/releases/tag/v0.5
