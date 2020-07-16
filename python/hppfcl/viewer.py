# Software License Agreement (BSD License)
#
#  Copyright (c) 2019 CNRS
#  Author: Joseph Mirabel

import hppfcl, numpy as np
from gepetto import Color

def applyConfiguration(gui, name, tf):
    gui.applyConfiguration(name, tf.getTranslation().tolist() + tf.getQuatRotation().coeffs().tolist())

def displayShape(gui, name, geom, color = (.9, .9, .9, 1.)):
    if isinstance(geom, hppfcl.Capsule):
        return gui.addCapsule(name, geom.radius, 2. * geom.halfLength, color)
    elif isinstance(geom, hppfcl.Cylinder):
        return gui.addCylinder(name, geom.radius, 2. * geom.halfLength, color)
    elif isinstance(geom, hppfcl.Box):
        w, h, d = (2. * geom.halfSide).tolist()
        return gui.addBox(name, w, h, d, color)
    elif isinstance(geom, hppfcl.Sphere):
        return gui.addSphere(name, geom.radius, color)
    elif isinstance(geom, hppfcl.Cone):
        return gui.addCone(name, geom.radius, 2. * geom.halfLength, color)
    elif isinstance(geom, hppfcl.Convex):
        pts = [ geom.points(geom.polygons(f)[i]).tolist() for f in range(geom.num_polygons) for i in range(3) ]
        gui.addCurve(name, pts, color)
        gui.setCurveMode(name, "TRIANGLES")
        gui.setLightingMode(name, "ON")
        gui.setBoolProperty(name, "BackfaceDrawing", True)
        return True
    elif isinstance(geom, hppfcl.ConvexBase):
        pts = [ geom.points(i).tolist() for i in range(geom.num_points) ]
        gui.addCurve(name, pts, color)
        gui.setCurveMode(name, "POINTS")
        gui.setLightingMode(name, "OFF")
        return True
    else:
        msg = "Unsupported geometry type for %s (%s)" % (geometry_object.name, type(geom) )
        warnings.warn(msg, category=UserWarning, stacklevel=2)
        return False

def displayDistanceResult(gui, group_name, res, closest_points = True, normal = True):
    gui.createGroup(group_name)
    r = 0.01
    if closest_points:
        p = [ group_name+"/p1", group_name+"/p2" ]
        gui.addSphere(p[0], r, Color.red)
        gui.addSphere(p[1], r, Color.blue)
        qid = [0,0,0,1]
        gui.applyConfigurations(p, [ res.getNearestPoint1().tolist() + qid, res.getNearestPoint2().tolist() + qid, ])
    if normal:
        n = group_name+"/normal"
        gui.addArrow(n, r, 0.1, Color.green)
        gui.applyConfiguration(n,
                res.getNearestPoint1().tolist() + hppfcl.Quaternion.FromTwoVectors(np.array([1,0,0]), res.normal).coeffs().tolist())
    gui.refresh()
