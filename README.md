HPP-FCL — An extension of the Flexible Collision Library
=======

<p align="center">
  <a href="https://travis-ci.org/humanoid-path-planner/hpp-fcl"><img src="https://travis-ci.org/humanoid-path-planner/hpp-fcl.svg?branch=master" alt="Building status"/></a>
  <a href="https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/commits/master/"><img src="https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/pipeline.svg" alt="Pipeline status"/></a>
  <a href="http://projects.laas.fr/gepetto/doc/humanoid-path-planner/hpp-fcl/master/coverage/"><img src="https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/coverage.svg?job=doc-coverage" alt="Coverage report"/></a>
  <a href="https://anaconda.org/conda-forge/hpp-fcl"><img src="https://img.shields.io/conda/dn/conda-forge/hpp-fcl.svg" alt="Conda Downloads"/></a>
  <a href="https://anaconda.org/conda-forge/hpp-fcl"><img src="https://img.shields.io/conda/vn/conda-forge/hpp-fcl.svg" alt="Conda Version"/></a>
  <a href="https://conda.anaconda.org/conda-forge"><img src="https://anaconda.org/conda-forge/hpp-fcl/badges/installer/conda.svg" alt="Anaconda-Server Badge"/></a>
</p>

This project is initially a fork from https://github.com/flexible-collision-library/fcl and has evolved since then.
The main new features are:
- the use of a safety margin when detecting collision,
- the computation of a lower bound of the distance between two objects when collision checking is performed and no collision is found.
- the implementation of Python bindings for easy code prototyping.
- the fix of various bugs.

This project is now used in many robotics frameworks such as [Pinocchio](https://github.com/stack-of-tasks/pinocchio), an open-source software which implements efficient and versatile rigid body dynamics algorithms and the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc), an open-source software for Motion and Manipulation Planning.

## Acknowledgments

The development of **HPP-FCL** is actively supported by the [Gepetto team](http://projects.laas.fr/gepetto/) [@LAAS-CNRS](http://www.laas.fr) and the [Willow team](https://www.di.ens.fr/willow/) [@INRIA](http://www.inria.fr).
