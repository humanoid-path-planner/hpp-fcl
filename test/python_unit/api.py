import unittest
from test_case import TestCase
import hppfcl
hppfcl.switchToNumpyArray()
import numpy as np

class TestMainAPI(TestCase):

    def test_collision(self):
        capsule = hppfcl.Capsule(1.,2.)
        M1 = hppfcl.Transform3f()
        M2 = hppfcl.Transform3f(np.eye(3),np.array([3, 0, 0]))

        req=hppfcl.CollisionRequest()
        res=hppfcl.CollisionResult()

        self.assertTrue(not hppfcl.collide(capsule, M1, capsule, M2, req, res))

    def test_distance(self):
        capsule = hppfcl.Capsule(1.,2.)
        M1 = hppfcl.Transform3f()
        M2 = hppfcl.Transform3f(np.eye(3),np.array([3, 0, 0]))

        req=hppfcl.DistanceRequest()
        res=hppfcl.DistanceResult()

        self.assertTrue(hppfcl.distance(capsule, M1, capsule, M2, req, res) > 0)

if __name__ == '__main__':
    unittest.main()
