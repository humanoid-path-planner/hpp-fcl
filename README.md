HPP-FCL — An extension of the Flexible Collision Library
=======

[![Building Status](https://travis-ci.org/humanoid-path-planner/hpp-fcl.svg?branch=master)](https://travis-ci.org/humanoid-path-planner/hpp-fcl)
[![Pipeline status](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/humanoid-path-planner/hpp-fcl/master/coverage/)

This project is initially a fork from https://github.com/flexible-collision-library/fcl and has evolved since then.
The main new features are:
- the use of a safety margin when detecting collision,
- the computation of a lower bound of the distance between two objects when collision checking is performed and no collision is found.
- the implementation of Python bindings for easy code prototyping.
- the fix of various bugs.

This project is now used in many robotics frameworks such as [Pinocchio](https://github.com/stack-of-tasks/pinocchio), an open-source software which implements efficient and versatile rigid body dynamics algorithms and the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc), an open-source software for Motion and Manipulation Planning.

## Acknowledgments

The development of **HPP-FCL** is actively supported by the [Gepetto team](http://projects.laas.fr/gepetto/) [@LAAS-CNRS](http://www.laas.fr)
