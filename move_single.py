from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()
import time
from openravepy import *
from openravepy.ikfast import *
import numpy as np
from pdb import set_trace as bp
import os.path
import time
from scipy.interpolate import interp1d
import sys

IS_MOVE = bool(int(sys.argv[1]))    # 0 reach, 1 move
IS_LARGE_OBS = (len(sys.argv)>2) and bool(sys.argv[2]=="l")

print("is move", IS_MOVE)
print("is large obstacles", IS_LARGE_OBS) 

env = Environment()
# env.SetViewer('qtcoin')
urdf_module = RaveCreateModule(env, 'urdf')

if IS_MOVE:
    urdf_path = "/data/or_planning_scripts/inmoov_arm_v2_2_moving_BB.urdf"
else:
    urdf_path = "/data/or_planning_scripts/inmoov_arm_v2_2_reaching_BB.urdf"

srdf_path = "/data/or_planning_scripts/inmoov_shadow_hand_v2.srdf"
np.set_printoptions(formatter={'int_kind': '{:,}'.format})
inmoov_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
robot = env.GetRobots()[0]
baseT_translation = np.array([-0.30, 0.348, 0.272])
BaseT = np.array([[1,0,0,baseT_translation[0]],
                [0,1,0,baseT_translation[1]],
                [0,0,1,baseT_translation[2]],
                [0,0,0,1]])
robot.SetTransform(BaseT)

if IS_MOVE:
    table = env.ReadKinBodyXMLFile('tabletop_move.kinbody.xml')
    env.Add(table)      # TODO: moved table down several cm
else:
    table = env.ReadKinBodyXMLFile('tabletop_reach.kinbody.xml')
    env.Add(table)      # TODO: moved table down 2 cm, move -x 2cm


manip = robot.SetActiveManipulator('right_arm')
#print(manip.GetArmIndices()) # [ 3  2  4  0  1 6 5] out of 7
RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug
# ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
# if not ikmodel.load():
#     ikmodel.autogenerate()

# palm pos/orient in object reference frame (use utility file to calculate matrix)
Apo = np.array([[ 7.96326711e-04, -9.73847322e-01,  2.27202023e-01,
    -1.80000000e-01],
    [ 0.00000000e+00, -2.27202095e-01, -9.73847631e-01,
        1.05000000e-01],
    [ 9.99999683e-01,  7.75500881e-04, -1.80927097e-04,
        1.30000000e-01],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.00000000e+00]])

#########################################################################################################################################
# listen for file and open it when it appears
file_path = '/data/PB_MOVE.npz' if IS_MOVE else '/data/PB_REACH.npz'

Qinit = [0.0]*7
Qdestin = [0.0]*7
while not os.path.exists(file_path):
    time.sleep(0.02)
if os.path.isfile(file_path):
    time.sleep(0.3)         # TODO: wait for networking
    try:
        loaded_data = np.load(file_path)
        OBJECTS = loaded_data['arr_0']
        if IS_MOVE:
            Qinit = loaded_data['arr_1']
            Qdestin = loaded_data['arr_2']
        else:
            Qdestin = loaded_data['arr_1']
        os.remove(file_path)
    except Exception:
        os.remove(file_path)
else:
    raise ValueError("%s isn't a file!" % file_path)

# robot.SetDOFValues(Qinit[0:7],[3, 2, 4, 0, 1, 6, 5])

# Qinit_or = np.array(Qinit)
# Qinit_or[[3, 2, 4, 0, 1, 6, 5]] = Qinit

# INPUT: Array with [x,y,z,theta] information in world coordinates for the destination location and obstacles. Destination should be entered in the first column. Also, is_box Boolean indicates whether target object is box or not.
#OBJECTS = np.array([[0.18, -0.18, 0.0, 0*np.pi/180],  
#                [0.0,-0.2, 0.0, 0*np.pi/180],   
#                [0.4, 0.3, 0.0, 0*np.pi/180],
#                [0.0, 0.3, 0.0, 0*np.pi/180]])



# Calculate transformation matrices for obstacles and target object
def get_transf_mat_from_pos_orient(xyz,theta):
    return np.array([[np.cos(theta),-np.sin(theta),0,xyz[0]],[np.sin(theta),np.cos(theta),0,xyz[1]],[0,0,1,xyz[2]],[0,0,0,1]])   
Tobj = []
start = 1 if IS_MOVE else 0
for i in range(start,OBJECTS.shape[0]):
    xyz = OBJECTS[i,0:3]
    theta = OBJECTS[i,-1]
    Tobj.append(get_transf_mat_from_pos_orient(xyz,theta))

print(Tobj)
# obs = []
# filename = "obs.kinbody.xml"
# for i in range(0,len(Tobj)): 
#     obs.append(env.ReadKinBodyXMLFile(filename))
#     env.Add(obs[-1])
#     obs[-1].SetTransform(Tobj[i])
#     print("aaa", i)

for i in range(0,len(Tobj)):        # TODO: <=6 objs
        if IS_LARGE_OBS:
            kinbody = 'obs%s_l.kinbody.xml' % i
        else:
            kinbody = 'obs%s.kinbody.xml' % i
        exec "obs%s=env.ReadKinBodyXMLFile(kinbody)" % i
        exec "env.Add(obs%s)" % i
        exec "obs%s.SetTransform(Tobj[i])" % i
        print("aaa")

# raw_input('press any key 3')

        
start_time = time.time()

# robot.SetDOFValues(Qinit[0:7],[3, 2, 4, 0, 1, 6, 5])
# print(robot.GetActiveDOFValues())

try:
    with robot:
        robot.SetDOFValues(Qinit[0:7],[3, 2, 4, 0, 1, 6, 5])
        # robot.SetDOFVelocities([0.0]*7,[3, 2, 4, 0, 1, 6, 5])
        # # robot.SetDOFValues(Qinit[0:7],[3, 2, 4, 0, 1, 6, 5])
        # robot.SetActiveDOFValues([0.0]*7)
        # robot.SetActiveDOFVelocities([0.0]*7)
        manipprob = interfaces.BaseManipulation(robot, maxvelmult=1.0) # create the interface for basic manipulation programs
        if IS_MOVE:
            res = manipprob.MoveManipulator(goal=Qdestin,outputtrajobj=True, jitter=0.2, execute=True) # call motion planner
        else:
            res = manipprob.MoveManipulator(goal=Qdestin,outputtrajobj=True, execute=False) # call motion planner
    traj = res.GetAllWaypoints2D()[:,0:-1]
    spec = res.GetConfigurationSpecification()
    # raw_input('press any key 4')
except PlanningError:
    # print(e)
    traj = []
end_time = time.time()
print("Duration: %.2f sec" % (end_time-start_time))

if len(traj) == 0:
    Traj_I = np.array([])
    Traj_S = np.array([])
else:
    n = 400 #interpolated trajectory resolution
    t = np.cumsum(traj[:,-1])
    T = np.linspace(t[0],t[-1],n)
    Traj_I = np.zeros((n,traj.shape[1]-8))      # TODO
    Traj_S = np.zeros((n,traj.shape[1]-8))      # TODO
    for i in range(traj.shape[1]-8):
        f = interp1d(t,traj[:,i],kind='linear')
        Traj_I[:,i] = f(T)
    
    idx = 0
    for ts in T:
        with env:
            trajdata = res.Sample(ts)
            Traj_S[idx, :] = spec.ExtractJointValues(trajdata,robot,[3, 2, 4, 0, 1, 6, 5],0)
            idx += 1

# for test in [0,1,2,3,4,10]:
#     print(Traj_I[test, :])
#     print(Traj_S[test, :])
#     print(T[test])

save_path = '/data/OR_MOVE.npz' if IS_MOVE else '/data/OR_REACH.npz'
np.savez(save_path, Traj_I, Traj_S)
#bp()

# raw_input('press any key 4')

# keep rerunning this script until it's terminated from the terminal
os.execv(sys.executable, ['python'] +sys.argv)