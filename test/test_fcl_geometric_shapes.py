# Datas for compare_convex_box
from gepetto.corbaserver import Client
from gepetto import Quaternion

def translate (tr, t, d):
    return [ tr[i] + d*t[i] for i in range(3) ] + tr[3:]

cl = Client ()
try:
    cl.gui.getWindowID("fcl")
except:
    cl.gui.createWindow("fcl")

cl.gui.addBox ('fcl/b0', 2, 2, 2, [1,0,0,0.5])
cl.gui.addBox ('fcl/b1', 2, 2, 2, [0,1,0,0.5])
cl.gui.setWireFrameMode ('fcl/b1', "WIREFRAME")
cl.gui.addBox ('fcl/b1_0', 2, 2, 2, [0,0  ,1,0.5])
cl.gui.addBox ('fcl/b1_1', 2, 2, 2, [0,0.5,1,0.5])

cl.gui.addSphere ("fcl/p0", 0.01, [1, 0, 1, 1])
cl.gui.addSphere ("fcl/p1", 0.01, [0, 1, 1, 1])

cl.gui.addArrow ("fcl/n0", 0.01, 1., [1, 0, 1, 1])
cl.gui.addArrow ("fcl/n1", 0.01, 1., [0, 1, 1, 1])

eps = 0.
d0 = 1.5183589910964868 + eps
n0 = [0.0310588, 0.942603, -0.332467]
d1 = 1.7485932899646754 + eps
n1 = [0.132426, -0.0219519, -0.99095]

qn0 = Quaternion()
qn1 = Quaternion()
qn0.fromTwoVector([1,0,0], n0)
qn1.fromTwoVector([1,0,0], n1)

pb1 = [ 0.135584, 0.933659, 0.290395, 0.119895, 0.977832, -0.164725, 0.0483272 ]
pb1_0 = translate (pb1, n0, d0)
pb1_1 = translate (pb1, n1, -d1)
cl.gui.applyConfiguration ("fcl/b1", pb1)
cl.gui.applyConfiguration ("fcl/b1_0", pb1_0)
cl.gui.applyConfiguration ("fcl/b1_1", pb1_1)

cl.gui.applyConfigurations(["fcl/p0","fcl/p1"], [
    [0.832569, 0.259513, -0.239598, 0,0,0,1],
    [-0.879579, 0.719545, 0.171906, 0,0,0,1] ])
cl.gui.applyConfigurations(["fcl/n0","fcl/n1"], [
    ( 0.832569, 0.259513, -0.239598, ) + qn0.toTuple(),
    ( -0.879579, 0.719545, 0.171906, ) + qn1.toTuple() ])

cl.gui.refresh()
