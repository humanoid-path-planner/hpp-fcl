# This script displays two triangles and two spheres in gepetto-gui
# It is useful to debug distance computation between two triangles.
import numpy as np
from gepetto.corbaserver import Client

Red = [1, 0, 0, .5]
Green = [0, 1, 0, .5]
Blue = [0, 0, 1, .5]

c = Client ()
wid = len (c.gui.getWindowList ()) - 1

sceneName = 'scene/triangles'
if sceneName in c.gui.getNodeList ():
    c.gui.deleteNode (sceneName, True)

wid = c.gui.createWindow ('triangles')

c.gui.createScene (sceneName)
c.gui.addSceneToWindow (sceneName, wid)

P1 = (-0.6475786872429674, -0.519875255189778, 0.5955961037406681)
P2 = (0.069105957031249998, -0.150722900390625, -0.42999999999999999)
P3 = (0.063996093749999997, -0.15320971679687501, -0.42999999999999999)
Q1 = (-25.655000000000001, -1.2858199462890625, 3.7249809570312502)
Q2 = (-10.926, -1.284259033203125, 3.7281499023437501)
Q3 = (-10.926, -1.2866180419921875, 3.72335400390625)
tf1 = (-12.824601270753471, -1.6840516940066426, 3.8914453043793844,
       -0.26862477561450587, -0.46249645019513175, 0.73064726592483387,
       -0.42437287410898855)
tf2 = (0, 0, 0, 0, 0, 0, 1)

c.gui.addTriangleFace ("triangle1", P1, P2, P3, Red)
c.gui.addTriangleFace ("triangle2", Q1, Q2, Q3, Green)
c.gui.addToGroup ('triangle1', sceneName)
c.gui.addToGroup ('triangle2', sceneName)

c.gui.applyConfiguration ('triangle1', tf1)
c.gui.applyConfiguration ('triangle2', tf2)
c.gui.refresh ()

