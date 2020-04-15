from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()
from openravepy import *
import numpy as np

env = Environment()
env.SetViewer('qtcoin')

urdf_path = "/data/or_planning_scripts/inmoov_arm_v2_2_moving_BB.urdf"
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

table = env.ReadKinBodyXMLFile('tabletop_move_tmp.kinbody.xml')
env.Add(table)      # TODO: moved table down several cm

manip = robot.SetActiveManipulator('right_arm')
RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug

# file_path = '/data/PB_MOVE_issue.npz'
# loaded_data = np.load(file_path)
# OBJECTS = loaded_data['arr_0']
# Qinit = loaded_data['arr_1']
# Qdestin = loaded_data['arr_2']

# print(Qinit)
# print(Qdestin)
Qinit = [-0.62254187, -0.19955548 , 0.18631672, -1.76567808 ,-0.73666037 ,
 0.07903109, 0.08479821]
Qdestin = [-0.40216814, -0.71749565,  0.11155115, -2.13895329, -0.97263565,
 -0.86788262, -0.23124707]

robot.SetDOFValues(Qinit,[3, 2, 4, 0, 1, 6, 5])
# print(robot.GetActiveDOFValues())
print(robot.GetDOFValues([3, 2, 4, 0, 1, 6, 5]))

raw_input('press any key')

try:
    manipprob = interfaces.BaseManipulation(robot, maxvelmult=1.0) # create the interface for basic manipulation programs
    res = manipprob.MoveManipulator(goal=Qdestin,outputtrajobj=True, jitter=0.05, execute=True) # call motion planner
except PlanningError as e:
    print(e)
    traj = []

robot.WaitForController(0)