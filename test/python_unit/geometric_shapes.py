import unittest
import hppfcl
hppfcl.switchToNumpyMatrix()
import numpy as np

class TestGeometricShapes(unittest.TestCase):

    def test_capsule(self):
        capsule = hppfcl.Capsule(1.,2.)
        self.assertIsInstance(capsule, hppfcl.Capsule)
        self.assertIsInstance(capsule, hppfcl.ShapeBase)
        self.assertIsInstance(capsule, hppfcl.CollisionGeometry)
        self.assertEqual(capsule.getNodeType(), hppfcl.NODE_TYPE.GEOM_CAPSULE)
        self.assertEqual(capsule.radius,1.)
        self.assertEqual(capsule.halfLength,1.)
        capsule.radius = 3.
        capsule.halfLength = 4.
        self.assertEqual(capsule.radius,3.)
        self.assertEqual(capsule.halfLength,4.)

    def test_box1(self):
        box = hppfcl.Box(np.matrix([1.,2.,3.]).T)
        self.assertIsInstance(box, hppfcl.Box)
        self.assertIsInstance(box, hppfcl.ShapeBase)
        self.assertIsInstance(box, hppfcl.CollisionGeometry)
        self.assertEqual(box.getNodeType(), hppfcl.NODE_TYPE.GEOM_BOX)
        self.assertTrue(np.array_equal(box.halfSide,np.matrix([.5,1.,1.5]).T))
        box.halfSide = np.matrix([4.,5.,6.]).T
        self.assertTrue(np.array_equal(box.halfSide,np.matrix([4.,5.,6.]).T))

    def test_box2(self):
        box = hppfcl.Box(1.,2.,3)
        self.assertIsInstance(box, hppfcl.Box)
        self.assertIsInstance(box, hppfcl.ShapeBase)
        self.assertIsInstance(box, hppfcl.CollisionGeometry)
        self.assertEqual(box.getNodeType(), hppfcl.NODE_TYPE.GEOM_BOX)
        self.assertEqual(box.halfSide[0],0.5)
        self.assertEqual(box.halfSide[1],1.0)
        self.assertEqual(box.halfSide[2],1.5)
        box.halfSide[0] = 4.
        box.halfSide[0] = 5.
        box.halfSide[0] = 6.
#         self.assertEqual(box.halfSide[0],4.)
#         self.assertEqual(box.halfSide[1],5.)
#         self.assertEqual(box.halfSide[2],6.)

    def test_sphere(self):
        sphere = hppfcl.Sphere(1.)
        self.assertIsInstance(sphere, hppfcl.Sphere)
        self.assertIsInstance(sphere, hppfcl.ShapeBase)
        self.assertIsInstance(sphere, hppfcl.CollisionGeometry)
        self.assertEqual(sphere.getNodeType(), hppfcl.NODE_TYPE.GEOM_SPHERE)
        self.assertEqual(sphere.radius,1.)
        sphere.radius = 2.
        self.assertEqual(sphere.radius,2.)

    def test_cylinder(self):
        cylinder = hppfcl.Cylinder(1.,2.)
        self.assertIsInstance(cylinder, hppfcl.Cylinder)
        self.assertIsInstance(cylinder, hppfcl.ShapeBase)
        self.assertIsInstance(cylinder, hppfcl.CollisionGeometry)
        self.assertEqual(cylinder.getNodeType(), hppfcl.NODE_TYPE.GEOM_CYLINDER)
        self.assertEqual(cylinder.radius,1.)
        self.assertEqual(cylinder.halfLength,1.)

    def test_cone(self):
        cone = hppfcl.Cone(1.,2.)
        self.assertIsInstance(cone, hppfcl.Cone)
        self.assertIsInstance(cone, hppfcl.ShapeBase)
        self.assertIsInstance(cone, hppfcl.CollisionGeometry)
        self.assertEqual(cone.getNodeType(), hppfcl.NODE_TYPE.GEOM_CONE)
        self.assertEqual(cone.radius,1.)
        self.assertEqual(cone.halfLength,1.)
        cone.radius = 3.
        cone.halfLength = 4.
        self.assertEqual(cone.radius,3.)
        self.assertEqual(cone.halfLength,4.)

if __name__ == '__main__':
    unittest.main()