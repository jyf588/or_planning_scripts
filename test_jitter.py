from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()
from openravepy import *
import numpy as np

env = Environment()
env.SetViewer('qtcoin')

urdf_path = "/data/or_planning_scripts/inmoov_arm_v2_2_reaching_BB.urdf"
srdf_path = "/data/or_planning_scripts/inmoov_shadow_hand_v2.srdf"
baseT_translation = np.array([-0.30, 0.348, 0.272])
BaseT = np.array([[1,0,0,baseT_translation[0]],
                [0,1,0,baseT_translation[1]],
                [0,0,1,baseT_translation[2]],
                [0,0,0,1]])


urdf_module = RaveCreateModule(env, 'urdf')

inmoov_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
robot = env.GetRobots()[0]

robot.SetTransform(BaseT)

table = env.ReadKinBodyXMLFile('tabletop_reach_tmp.kinbody.xml')
env.Add(table)      # TODO: moved table down 2 cm, move -x 2cm


manip = robot.SetActiveManipulator('right_arm')
#print(manip.GetArmIndices()) # [ 3  2  4  0  1 6 5] out of 7
RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug

# Qinit = [-0.62254187, -0.19955548,  0.18631672, -1.76567808, -0.73666037,  0.07903109,
#   0.08479821]
Qinit = [0.0]*5 + [-1.0] + [0.1]
Qdestin = [-1.0912841014949577, -0.009655931223847022, -0.7928532573212783, -1.5623178881531612, -1.001228623703925, -0.5604086771696383, 0.3747606991248253]

robot.SetDOFValues(Qinit,[3, 2, 4, 0, 1, 6, 5])
# robot.SetActiveDOFValues(Qinit)
print(robot.GetDOFValues([3, 2, 4, 0, 1, 6, 5]))

raw_input('press any key')

try:
    manipprob = interfaces.BaseManipulation(robot, maxvelmult=1.0) # create the interface for basic manipulation programs
    res = manipprob.MoveManipulator(goal=Qdestin,outputtrajobj=True, jitter=0.2, execute=True) # call motion planner
except PlanningError as e:
    print(e)
    traj = []

# with robot:
#     # robot.SetActiveDOFValues([0.0]*7)
#     # robot.SetActiveDOFVelocities([0.0]*7)


robot.WaitForController(0) # wait