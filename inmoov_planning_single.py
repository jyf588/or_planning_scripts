from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()
import time
from openravepy import *
from openravepy.ikfast import *
import numpy as np
from pdb import set_trace as bp

env = Environment()
env.SetViewer('qtcoin')
urdf_module = RaveCreateModule(env, 'urdf')

urdf_path = "package://inmoov_description/robots/inmoov_shadow_hand_v2.urdf"
srdf_path = "package://inmoov_description/srdf/inmoov_shadow_hand_v2.srdf"
np.set_printoptions(formatter={'int_kind': '{:,}'.format})
inmoov_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
print('robot name:', inmoov_name)
robot = env.GetRobots()[0]
print(robot)
# table = env.ReadKinBodyXMLFile('tabletop.kinbody.xml')
# env.Add(table)
#baseToPalmOffset = np.array([-0.27157567, 0.37579589,  -1.07620252])
#palm_init_pos=np.array([-0.17, 0.07, 0.1])
#baseT_translation = baseToPalmOffset + palm_init_pos
baseT_translation = np.array([-0.3, 0.5, -1.25])
BaseT = np.array([[1,0,0,baseT_translation[0]],
                [0,1,0,baseT_translation[1]],
                [0,0,1,baseT_translation[2]],
                [0,0,0,1]])
robot.SetTransform(BaseT)


           
table = env.ReadKinBodyXMLFile('tabletop_2.kinbody.xml')
env.Add(table)
obs1 = env.ReadKinBodyXMLFile('obs1.kinbody.xml')
env.Add(obs1)
target = env.ReadKinBodyXMLFile('target.kinbody.xml')
env.Add(target)

manip = robot.SetActiveManipulator('right_arm')
#print(manip.GetArmIndices()) # [ 8  7  9  5  6 33 32] out of 34
RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()
robot.SetDOFValues([0.0]*34,range(34))     # 5*2+2+22



########## enter object location and orientation in world coordinate frame
obj_xyz = np.array([0.10, 0.10, 0])
obj_theta = 45*np.pi/180 #rotation is assumed to be around z-axis
is_box = True
Theta_min = 0 # choose interval Theta_min - Theta_max for wrist orientation wrt the object if it is not a box
Theta_max = 2*np.pi/3

choose = 1 # choose one of those options to plan and execute 

# calculate transformation matrix
RotMat_w_o = np.array([[np.cos(obj_theta),-np.sin(obj_theta),0,obj_xyz[0]],[np.sin(obj_theta),np.cos(obj_theta),0,obj_xyz[1]],[0,0,1,obj_xyz[2]],[0,0,0,1]]) # calculate rotation matrix
h = env.plot3(RotMat_w_o[0:3,3],30)
raw_input('press any key 3')
# palm pos/orient in object reference frame (use utility file to calculate matrix)
Apo = np.array([[ 7.96326711e-04, -9.73847322e-01,  2.27202023e-01,
    -1.80000000e-01],
    [ 0.00000000e+00, -2.27202095e-01, -9.73847631e-01,
        1.05000000e-01],
    [ 9.99999683e-01,  7.75500881e-04, -1.80927097e-04,
        1.30000000e-01],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.00000000e+00]])


if is_box:
   Theta = np.array([-np.pi/2,0,np.pi/2])
else:
    Theta = np.linspace(Theta_min,Theta_max,num=3)  # choose a few wrist orientations in the interval 0 to 90

print 'Wrist rotation angles in deg: ' , Theta*180/np.pi
Tgoal=[]
for theta in Theta:
        RotMat = np.array([[np.cos(theta),-np.sin(theta),0,0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]]) # calculate rotation matrix
        Apo_varried = np.matmul(RotMat,Apo) # get varied Apo
        Tgoal.append(np.matmul(RotMat_w_o,Apo_varried)) # transform to world coordinate frame

        
h = env.plot3(Tgoal[choose][0:3,3],30)

raw_input('press any key 3')
start_time = time.time()
robot.SetDOFValues([0.0]*34,range(34))
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
try:
    res = manipprob.MoveToHandPosition(matrices=[Tgoal[choose]],seedik=10,outputtrajobj=True) # call motion planner with goal ee 6D
    traj = res.GetAllWaypoints2D()[:,0:-1]
except:
    traj = []
end_time = time.time()
print("Duration: %.2f sec" % (end_time-start_time))
bp()

