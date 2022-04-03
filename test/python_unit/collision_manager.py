import hppfcl as fcl
import numpy as np

sphere = fcl.Sphere(0.5)
sphere_obj = fcl.CollisionObject(sphere)

M_sphere = fcl.Transform3f.Identity()
M_sphere.setTranslation(np.array([-0.6, 0.0, 0.0]))
sphere_obj.setTransform(M_sphere)

box = fcl.Box(np.array([0.5, 0.5, 0.5]))
box_obj = fcl.CollisionObject(box)

M_box = fcl.Transform3f.Identity()
M_box.setTranslation(np.array([-0.6, 0.0, 0.0]))
box_obj.setTransform(M_box)

collision_manager = fcl.DynamicAABBTreeCollisionManager()
collision_manager.registerObject(sphere_obj)
collision_manager.registerObject(box_obj)

assert collision_manager.size() == 2

collision_manager.setup()

# Perform collision detection
callback = fcl.CollisionCallBackDefault()
collision_manager.collide(sphere_obj, callback)

assert callback.data.result.numContacts() == 1
