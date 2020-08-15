import unittest
from test_case import TestCase
import hppfcl
import numpy as np

class TestGeometricShapes(TestCase):

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
        com = capsule.computeCOM()
        self.assertApprox(com, np.zeros(3))
        V = capsule.computeVolume()
        V_cylinder = capsule.radius * capsule.radius * np.pi * 2. * capsule.halfLength
        V_sphere = 4. * np.pi/3 * capsule.radius**3
        V_ref = V_cylinder + V_sphere
        self.assertApprox(V, V_ref)
        I0 = capsule.computeMomentofInertia()
        Iz_cylinder = V_cylinder * capsule.radius**2 / 2.
        Iz_sphere = 0.4 * V_sphere * capsule.radius * capsule.radius
        Iz_ref = Iz_cylinder + Iz_sphere
        Ix_cylinder = V_cylinder*(3 * capsule.radius**2 + 4 * capsule.halfLength**2)/12.
        V_hemi = 0.5 * V_sphere                                          # volume of hemisphere
        I0x_hemi = 0.5 * Iz_sphere                                       # inertia of hemisphere w.r.t. origin
        com_hemi = 3. * capsule.radius / 8.                              # CoM of hemisphere w.r.t. origin
        Icx_hemi = I0x_hemi - V_hemi * com_hemi * com_hemi               # inertia of hemisphere w.r.t. CoM
        Ix_hemi = Icx_hemi + V_hemi * (capsule.halfLength + com_hemi)**2 # inertia of hemisphere w.r.t. tip of cylinder
        Ix_ref = Ix_cylinder + 2*Ix_hemi                                 # total inertia of capsule
        I0_ref = np.diag([Ix_ref,Ix_ref,Iz_ref])
        self.assertApprox(I0, I0_ref)
        Ic = capsule.computeMomentofInertiaRelatedToCOM()
        self.assertApprox(Ic, I0_ref)

    def test_box1(self):
        box = hppfcl.Box(np.array([1.,2.,3.]))
        self.assertIsInstance(box, hppfcl.Box)
        self.assertIsInstance(box, hppfcl.ShapeBase)
        self.assertIsInstance(box, hppfcl.CollisionGeometry)
        self.assertEqual(box.getNodeType(), hppfcl.NODE_TYPE.GEOM_BOX)
        self.assertTrue(np.array_equal(box.halfSide,np.array([.5,1.,1.5])))
        box.halfSide = np.array([4.,5.,6.])
        self.assertTrue(np.array_equal(box.halfSide,np.array([4.,5.,6.])))
        com = box.computeCOM()
        self.assertApprox(com, np.zeros(3))
        V = box.computeVolume()
        x = float(2*box.halfSide[0])
        y = float(2*box.halfSide[1])
        z = float(2*box.halfSide[2])
        V_ref = x * y * z
        self.assertApprox(V, V_ref)
        I0 = box.computeMomentofInertia()
        Ix = V_ref * (y*y + z*z) / 12.
        Iy = V_ref * (x*x + z*z) / 12.
        Iz = V_ref * (y*y + x*x) / 12.
        I0_ref = np.diag([Ix,Iy,Iz])
        self.assertApprox(I0, I0_ref)
        Ic = box.computeMomentofInertiaRelatedToCOM()
        self.assertApprox(Ic, I0_ref) 

    def test_box2(self):
        box = hppfcl.Box(1.,2.,3)
        self.assertIsInstance(box, hppfcl.Box)
        self.assertIsInstance(box, hppfcl.ShapeBase)
        self.assertIsInstance(box, hppfcl.CollisionGeometry)
        self.assertEqual(box.getNodeType(), hppfcl.NODE_TYPE.GEOM_BOX)
        self.assertEqual(box.halfSide[0],0.5)
        self.assertEqual(box.halfSide[1],1.0)
        self.assertEqual(box.halfSide[2],1.5)
 
    def test_sphere(self):
        sphere = hppfcl.Sphere(1.)
        self.assertIsInstance(sphere, hppfcl.Sphere)
        self.assertIsInstance(sphere, hppfcl.ShapeBase)
        self.assertIsInstance(sphere, hppfcl.CollisionGeometry)
        self.assertEqual(sphere.getNodeType(), hppfcl.NODE_TYPE.GEOM_SPHERE)
        self.assertEqual(sphere.radius, 1.)
        sphere.radius = 2.
        self.assertEqual(sphere.radius, 2.)
        com = sphere.computeCOM()
        self.assertApprox(com, np.zeros(3))
        V = sphere.computeVolume()
        V_ref = 4. * np.pi/3 * sphere.radius**3
        self.assertApprox(V, V_ref)
        I0 = sphere.computeMomentofInertia()
        I0_ref = 0.4 * V_ref * sphere.radius * sphere.radius * np.identity(3)
        self.assertApprox(I0, I0_ref)
        Ic = sphere.computeMomentofInertiaRelatedToCOM()
        self.assertApprox(Ic, I0_ref)

    def test_cylinder(self):
        cylinder = hppfcl.Cylinder(1.,2.)
        self.assertIsInstance(cylinder, hppfcl.Cylinder)
        self.assertIsInstance(cylinder, hppfcl.ShapeBase)
        self.assertIsInstance(cylinder, hppfcl.CollisionGeometry)
        self.assertEqual(cylinder.getNodeType(), hppfcl.NODE_TYPE.GEOM_CYLINDER)
        self.assertEqual(cylinder.radius,1.)
        self.assertEqual(cylinder.halfLength,1.)
        cylinder.radius = 3.
        cylinder.halfLength = 4.
        self.assertEqual(cylinder.radius,3.)
        self.assertEqual(cylinder.halfLength,4.)
        com = cylinder.computeCOM()
        self.assertApprox(com, np.zeros(3))
        V = cylinder.computeVolume()
        V_ref = cylinder.radius * cylinder.radius * np.pi * 2. * cylinder.halfLength
        self.assertApprox(V, V_ref)
        I0 = cylinder.computeMomentofInertia()
        Ix_ref = V_ref*(3 * cylinder.radius**2 + 4 * cylinder.halfLength**2)/12.
        Iz_ref = V_ref * cylinder.radius**2 / 2.
        I0_ref = np.diag([Ix_ref,Ix_ref,Iz_ref])
        self.assertApprox(I0, I0_ref)
        Ic = cylinder.computeMomentofInertiaRelatedToCOM()
        self.assertApprox(Ic, I0_ref)

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
        com = cone.computeCOM()
        self.assertApprox(com, np.array([0.,0.,-0.5 * cone.halfLength]))
        V = cone.computeVolume()
        V_ref = np.pi * cone.radius**2 * 2. * cone.halfLength / 3.
        self.assertApprox(V, V_ref)
        I0 = cone.computeMomentofInertia()
        Ix_ref = V_ref * (3./20. * cone.radius**2 + 0.4 * cone.halfLength**2)
        Iz_ref = 0.3 * V_ref * cone.radius**2
        I0_ref = np.diag([Ix_ref,Ix_ref,Iz_ref])
        self.assertApprox(I0, I0_ref)
        Ic = cone.computeMomentofInertiaRelatedToCOM()
        Icx_ref = V_ref * 3./20. * (cone.radius**2 + cone.halfLength**2)
        Ic_ref = np.diag([Icx_ref,Icx_ref,Iz_ref])
        self.assertApprox(Ic, Ic_ref)

    def test_convex(self):
        verts = hppfcl.StdVec_Vec3f ()
        faces = hppfcl.StdVec_Triangle ()
        verts.extend( [ np.array([0, 0, 0]), np.array([0, 1, 0]), np.array([1, 0, 0]), ])
        faces.append(hppfcl.Triangle(0,1,2))
        convex = hppfcl.Convex(verts, faces)

        verts.append (np.array([0, 0, 1]))
        try:
            convexHull = hppfcl.Convex.convexHull(verts, False, None)
            qhullAvailable = True
        except Exception as e:
            self.assertEqual(str(e), "Library built without qhull. Cannot build object of this type.")
            qhullAvailable = False

        if qhullAvailable:
            convexHull = hppfcl.Convex.convexHull(verts, False, "")
            convexHull = hppfcl.Convex.convexHull(verts, True, "")

            try:
                convexHull = hppfcl.Convex.convexHull(verts[:3], False, None)
            except Exception as e:
                self.assertIn(str(e), "You shouldn't use this function with less than 4 points.")

if __name__ == '__main__':
    unittest.main()
