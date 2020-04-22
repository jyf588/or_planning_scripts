# test planning inmoov shadow hand
from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()

from PyQt5 import QtGui, QtCore
import logging

import numpy as np

from openravepy import *
from openravepy.ikfast import *

def printDHParams():
    print "index, name, parent index, d, a, theta, alpha"
    for i,p in enumerate(planningutils.GetDHParameters(robot)):
        print "%d, %s, %d, %f, %f, %f, %f" % (i, p.joint.GetName(), p.parentindex, p.d, p.a, p.theta, p.alpha)

env = Environment()
# server = Server()
env.SetViewer('qtcoin')
# logger = logging.getLogger('PyqtControl')
urdf_module = RaveCreateModule(env, 'urdf')

urdf_path = "package://inmoov_description/robots/inmoov_shadow_hand_v2.urdf"
srdf_path = "package://inmoov_description/srdf/inmoov_shadow_hand_v2.srdf"

h = env.plot3([1,0,0],30)

inmoov_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
print('robot name:', inmoov_name)
robot = env.GetRobots()[0]
print(robot)
BaseT = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,-1],
                    [0,0,0,1]])
robot.SetTransform(BaseT)
# urdf_module.SendCommand('LoadURI {}'.format("tabletop.urdf"))

printDHParams()

manip = robot.SetActiveManipulator('right_arm')
print(manip.GetArmIndices()) # [ 8  7  9  5  6 33 32] out of 34

# robot.SetDOFValues([-0.5]*42,range(42))

RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug

raw_input('press any key 1')

ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()
raw_input('press any key 2')

# ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
# if not ikmodel.load():
#     ikmodel.autogenerate()

robot.SetDOFValues([0.0]*34,range(34))     # 5*2+2+22
robot.SetDOFValues([-0.5]*7, [8, 7, 9, 5, 6, 33, 32])  
Tgoal = manip.GetEndEffectorTransform() # get end effector # ,,
h = env.plot3(Tgoal[0:3,3],30)

robot.SetDOFValues([0.0]*34,range(34))
raw_input('press any key 3')
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal ee 6D
raw_input('press any key 4') # wait
