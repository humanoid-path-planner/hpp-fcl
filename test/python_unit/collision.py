import unittest
from test_case import TestCase
import hppfcl
hppfcl.switchToNumpyArray()
import numpy as np

def tetahedron():
    pts = hppfcl.StdVec_Vec3f()
    pts.append( np.array((0, 0, 0)) )
    pts.append( np.array((0, 1, 0)) )
    pts.append( np.array((1, 0, 0)) )
    pts.append( np.array((0, 0, 1)) )
    tri = hppfcl.StdVec_Triangle()
    tri.append(hppfcl.Triangle(0,1,2))
    tri.append(hppfcl.Triangle(0,1,3))
    tri.append(hppfcl.Triangle(0,2,3))
    tri.append(hppfcl.Triangle(1,2,3))
    return hppfcl.Convex(pts, tri)

class TestMainAPI(TestCase):

    def test_convex_halfspace(self):
        convex = tetahedron()
        halfspace = hppfcl.Halfspace(np.array((0,0,1)), 0)

        req=hppfcl.CollisionRequest()
        res=hppfcl.CollisionResult()

        M1 = hppfcl.Transform3f()
        M2 = hppfcl.Transform3f(np.eye(3),np.array([0, 0, -0.1]))
        res.clear()
        hppfcl.collide(convex, M1, halfspace, M2, req, res)
        self.assertFalse(hppfcl.collide(convex, M1, halfspace, M2, req, res))

        M1 = hppfcl.Transform3f()
        M2 = hppfcl.Transform3f(np.eye(3),np.array([0, 0, 0.1]))
        res.clear()
        self.assertTrue(hppfcl.collide(convex, M1, halfspace, M2, req, res))

        M1 = hppfcl.Transform3f()
        M2 = hppfcl.Transform3f(np.eye(3),np.array([0, 0, 2]))
        res.clear()
        self.assertTrue(hppfcl.collide(convex, M1, halfspace, M2, req, res))

if __name__ == '__main__':
    unittest.main()
