# fixed planning inmoov shadow hand
from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()

from openravepy import *
from openravepy.ikfast import *
import numpy as np

env = Environment()
env.SetViewer('qtcoin')
urdf_module = RaveCreateModule(env, 'urdf')

urdf_path = "package://inmoov_description/robots/inmoov_shadow_hand_v2.urdf"
srdf_path = "package://inmoov_description/srdf/inmoov_shadow_hand_v2.srdf"

inmoov_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
print('robot name:', inmoov_name)
robot = env.GetRobots()[0]
print(robot)
# table = env.ReadKinBodyXMLFile('tabletop.kinbody.xml')
# env.Add(table)
baseToPalmOffset = np.array([-0.27157567, 0.37579589,  -1.07620252])
palm_init_pos=np.array([-0.17, 0.07, 0.1])
baseT_translation = baseToPalmOffset + palm_init_pos
BaseT = np.array([[1,0,0,baseT_translation[0]],
                [0,1,0,baseT_translation[1]],
                [0,0,1,baseT_translation[2]],
                [0,0,0,1]])
robot.SetTransform(BaseT)

# table_name = urdf_module.SendCommand('LoadURI {}'.format("tabletop.urdf"))
# table_kinbody = env.GetKinBody(table_name)
# tableT = np.array([[1,0,0, 0.1],
#                 [0,1,0, 0],
#                 [0,0,1, -0.05],
#                 [0,0,0, 1]])
# table_kinbody.SetTransform(tableT)     
           
table = env.ReadKinBodyXMLFile('tabletop_2.kinbody.xml')
env.Add(table)

manip = robot.SetActiveManipulator('right_arm')
print(manip.GetArmIndices()) # [ 8  7  9  5  6 33 32] out of 34

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
# robot.SetDOFValues([-0.5]*7, [8, 7, 9, 5, 6, 33, 32])  
robot.SetDOFValues([-0.1766934, 0.0010606, -0.347004, -1.296158, -0.169959, 0.2112, -0.9534262],
                    [8, 7, 9, 5, 6, 33, 32]) 
Tgoal = manip.GetEndEffectorTransform() # get end effector # ,,
h = env.plot3(Tgoal[0:3,3],30)

robot.SetDOFValues([0.0]*34,range(34))
raw_input('press any key 3')
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal ee 6D
raw_input('press any key 4') # wait