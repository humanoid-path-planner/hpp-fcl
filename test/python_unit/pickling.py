import unittest
from test_case import TestCase
import hppfcl

import pickle
import numpy as np


def tetahedron():
    pts = hppfcl.StdVec_Vec3f()
    pts.append(np.array((0, 0, 0)))
    pts.append(np.array((0, 1, 0)))
    pts.append(np.array((1, 0, 0)))
    pts.append(np.array((0, 0, 1)))
    tri = hppfcl.StdVec_Triangle()
    tri.append(hppfcl.Triangle(0, 1, 2))
    tri.append(hppfcl.Triangle(0, 1, 3))
    tri.append(hppfcl.Triangle(0, 2, 3))
    tri.append(hppfcl.Triangle(1, 2, 3))
    return hppfcl.Convex(pts, tri)


class TestGeometryPickling(TestCase):
    def pickling(self, obj):
        with open("save.p", "wb") as f:
            pickle.dump(obj, f)
        with open("save.p", "rb") as f:
            obj2 = pickle.load(f)

        self.assertTrue(obj == obj2)

    def test_all_shapes(self):
        box = hppfcl.Box(1.0, 2.0, 3.0)
        self.pickling(box)

        sphere = hppfcl.Sphere(1.0)
        self.pickling(sphere)

        ellipsoid = hppfcl.Ellipsoid(1.0, 2.0, 3.0)
        self.pickling(ellipsoid)

        convex = tetahedron()
        self.pickling(convex)

        capsule = hppfcl.Capsule(1.0, 2.0)
        self.pickling(capsule)

        cylinder = hppfcl.Cylinder(1.0, 2.0)
        self.pickling(cylinder)

        plane = hppfcl.Plane(np.array([0.0, 0.0, 1.0]), 2.0)
        self.pickling(plane)

        half_space = hppfcl.Halfspace(np.array([0.0, 0.0, 1.0]), 2.0)
        self.pickling(half_space)


if __name__ == "__main__":
    unittest.main()
