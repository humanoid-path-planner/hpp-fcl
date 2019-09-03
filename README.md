## FCL -- The Flexible Collision Library

[![Building Status](https://travis-ci.org/humanoid-path-planner/hpp-fcl.svg?branch=master)](https://travis-ci.org/humanoid-path-planner/hpp-fcl)
[![Pipeline status](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-fcl/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/humanoid-path-planner/hpp-fcl/master/coverage/)

This project is a fork from https://github.com/flexible-collision-library/fcl.

The main differences are.
- the use of a safety margin when detecting collision,
- the computation of a lower bound of the distance between two objects when collision checking is performed and no collision is found.
