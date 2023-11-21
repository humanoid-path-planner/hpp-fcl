# HPP-FCL — An extension of the Flexible Collision Library

<p align="center">
  <a href="https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/commits/master/"><img src="https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/pipeline.svg" alt="Pipeline status"/></a>
  <a href="https://gepettoweb.laas.fr/hpp/hpp-fcl/doxygen-html/index.html"><img src="https://img.shields.io/badge/docs-online-brightgreen" alt="Documentation"/></a>
  <a href="http://projects.laas.fr/gepetto/doc/humanoid-path-planner/hpp-fcl/master/coverage/"><img src="https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/coverage.svg?job=doc-coverage" alt="Coverage report"/></a>
  <a href="https://anaconda.org/conda-forge/hpp-fcl"><img src="https://img.shields.io/conda/dn/conda-forge/hpp-fcl.svg" alt="Conda Downloads"/></a>
  <a href="https://anaconda.org/conda-forge/hpp-fcl"><img src="https://img.shields.io/conda/vn/conda-forge/hpp-fcl.svg" alt="Conda Version"/></a>
  <a href="https://badge.fury.io/py/hpp-fcl"><img src="https://badge.fury.io/py/hpp-fcl.svg" alt="PyPI version"></a>
  <a href="https://github.com/psf/black"><img alt="black" src="https://img.shields.io/badge/code%20style-black-000000.svg"></a>
  <a href="https://github.com/astral-sh/ruff"><img alt="ruff" src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json"></a>
</p>

[FCL](https://github.com/flexible-collision-library/fcl) was forked in 2015. Since then, a large part of the code has been rewritten or removed (for the unused and untested part).
The broad phase was reintroduced by J. Carpentier in 2022 based on the FCL version 0.7.0.

## New features

Compared to the original [FCL](https://github.com/flexible-collision-library/fcl) library, the main new features are:
- a dedicated and efficient implementation of the GJK algorithm (we do not rely anymore on [libccd](https://github.com/danfis/libccd))
- the support of safety margins for collision detection
- an accelerated version of collision detection *à la Nesterov*, which leads to increased performances (up to a factor of 2). More details are available in this [paper](https://hal.archives-ouvertes.fr/hal-03662157/)
- the computation of a lower bound of the distance between two objects when collision checking is performed, and no collision is found
- the implementation of Python bindings for easy code prototyping
- the support of height fields, capsule shapes, etc.
- the fix of various bugs

This project is now used in many robotics frameworks such as [Pinocchio](https://github.com/stack-of-tasks/pinocchio), an open-source software that implements efficient and versatile rigid body dynamics algorithms and the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc), an open-source software for Motion and Manipulation Planning.

## A high-performance library

Unlike the original FCL library, HPP-FCL implements the well-established [GJK algorithm](https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm) and [its variants](https://hal.archives-ouvertes.fr/hal-03662157/) for collision detection and distance computation. These implementations lead to state-of-the-art performances, as depicted by the figures below.

On the one hand, we have benchmarked HPP-FCL against major software alternatives of the state of the art:
1. the [Bullet simulator](https://github.com/bulletphysics/bullet3),
2. the original [FCL library](https://github.com/flexible-collision-library/fcl) (used in the [Drake framework]()),
3. the [libccd library](https://github.com/danfis/libccd) (used in [MuJoCo](http://mujoco.org/)).

The results are depicted in the following figure, which notably shows that the accelerated variants of GJK largely outperform by a large margin (from 5x up to 15x times faster).

<p align="center">
  <img src="./doc/images/hpp-fcl-vs-the-rest-of-the-world.png" width="600" alt="HPP-FCL vs the rest of the world" align="center"/>
</p>

On the other hand, why do we care about dedicated collision detection solvers like GJK for the narrow phase? Why can't we simply formulate the collision detection problem as a quadratic problem and call an off-the-shelf optimization solver like [ProxQP](https://github.com/Simple-Robotics/proxsuite))? Here is why.

<p align="center">
  <img src="./doc/images/hpp-fcl-performances.jpg" width="600" alt="HPP-FCL vs generic QP solvers" align="center"/>
</p>

One can observe that GJK-based approaches largely outperform solutions based on classic optimization solvers (e.g., QP solver like [ProxQP](https://github.com/Simple-Robotics/proxsuite)), notably for large geometries composed of tens or hundreds of vertices.

## Acknowledgments

The development of **HPP-FCL** is actively supported by the [Gepetto team](http://projects.laas.fr/gepetto/) [@LAAS-CNRS](http://www.laas.fr), the [Willow team](https://www.di.ens.fr/willow/) [@INRIA](http://www.inria.fr) and, to some extend, [Eureka Robotics](https://eurekarobotics.com/).
