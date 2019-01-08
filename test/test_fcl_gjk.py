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
P2 = (0.4653088233737781, 0.313127305970121, 0.934810277044219)
P3 = (0.2789166910941325, 0.5194696837661181, -0.8130390456938367)
Q1 = (-0.7301951766620367, 0.04042013969291935, -0.8435357165725602)
Q2 = (-0.8601872044895716, -0.5906898274974384, -0.07715905321629679)
Q3 = (0.6393545603562867, 0.1466372567911807, 0.5111616707924576)
p1 = (0.2238377827027012, 0.2163758468474661, 0.4505435657492071)
p2 = (0.2238377827027012, 0.2163758468474662, 0.450543565749207)
tf2 = (-0.1588705771755817, 0.1959319373363798, 0.04007284069698749) + (0, 0, 0, 1)
normal = (-0.6220181878092853, 0.7671227156255918, 0.1568952300284174)

c.gui.addTriangleFace ("triangle1", P1, P2, P3, Red)
c.gui.addTriangleFace ("triangle2", Q1, Q2, Q3, Green)
c.gui.addToGroup ('triangle1', sceneName)
c.gui.addToGroup ('triangle2', sceneName)

c.gui.addSphere ('closest point 1', .02, Red)
c.gui.addSphere ('closest point 2', .02, Green)
c.gui.addToGroup ('closest point 1', sceneName)
c.gui.addToGroup ('closest point 2', sceneName)

c.gui.addLine ('normal', P1, tuple (np.array (P1) + np.array (normal)), Red)
c.gui.addToGroup ('normal', sceneName)

q1 = p1 + (1,0,0,0)
q2 = p2 + (1,0,0,0)
tf1 = (0, 0, 0, 0, 0, 0, 1)

c.gui.applyConfiguration ('closest point 1', q1)
c.gui.applyConfiguration ('closest point 2', q2)
c.gui.applyConfiguration ('triangle1', tf1)
c.gui.applyConfiguration ('triangle2', tf2)
c.gui.refresh ()

