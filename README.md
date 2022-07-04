HPP-FCL — An extension of the Flexible Collision Library
=======

<p align="center">
  <a href="https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/commits/master/"><img src="https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/pipeline.svg" alt="Pipeline status"/></a>
  <a href="http://projects.laas.fr/gepetto/doc/humanoid-path-planner/hpp-fcl/master/coverage/"><img src="https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/coverage.svg?job=doc-coverage" alt="Coverage report"/></a>
  <a href="https://anaconda.org/conda-forge/hpp-fcl"><img src="https://img.shields.io/conda/dn/conda-forge/hpp-fcl.svg" alt="Conda Downloads"/></a>
  <a href="https://anaconda.org/conda-forge/hpp-fcl"><img src="https://img.shields.io/conda/vn/conda-forge/hpp-fcl.svg" alt="Conda Version"/></a>
  <a href="https://badge.fury.io/py/hpp-fcl"><img src="https://badge.fury.io/py/hpp-fcl.svg" alt="PyPI version"></a>
</p>

[FCL](https://github.com/flexible-collision-library/fcl) was forked in 2015. Since then, a large part of the code has been rewritten or removed (for the unused part).
The broadphase was reintroduced by J. Carpentier in 2022 based on the FCL version 0.7.0.

Compared to the original [FCL](https://github.com/flexible-collision-library/fcl) library, the main new features are:
- a dedicated implementation of the GJK algorithm (we do not rely anymore on [libccd](https://github.com/danfis/libccd))
- the use of a safety margin when detecting collision
- an accelerated version of Collision Detection *à la Nesterov* which leads to increased performances (up to a factor 2). More details are available in this [paper](https://hal.archives-ouvertes.fr/hal-03662157/)
- the computation of a lower bound of the distance between two objects when collision checking is performed and no collision is found
- the implementation of Python bindings for easy code prototyping
- the fix of various bugs

This project is now used in many robotics frameworks such as [Pinocchio](https://github.com/stack-of-tasks/pinocchio), an open-source software which implements efficient and versatile rigid body dynamics algorithms and the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc), an open-source software for Motion and Manipulation Planning.

## Acknowledgments

The development of **HPP-FCL** is actively supported by the [Gepetto team](http://projects.laas.fr/gepetto/) [@LAAS-CNRS](http://www.laas.fr), the [Willow team](https://www.di.ens.fr/willow/) [@INRIA](http://www.inria.fr) and, to some extend, [Eureka Robotics](https://eurekarobotics.com/).

