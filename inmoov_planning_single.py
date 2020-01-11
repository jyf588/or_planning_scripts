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
urdf_path = "package://inmoov_description/robots/inmoov_shadow_hand_v2_1.urdf"
srdf_path = "package://inmoov_description/srdf/inmoov_shadow_hand_v2.srdf"
np.set_printoptions(formatter={'int_kind': '{:,}'.format})
inmoov_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
robot = env.GetRobots()[0]
baseT_translation = np.array([-0.3, 0.5, -1.25])
BaseT = np.array([[1,0,0,baseT_translation[0]],
                [0,1,0,baseT_translation[1]],
                [0,0,1,baseT_translation[2]],
                [0,0,0,1]])
robot.SetTransform(BaseT)


           
table = env.ReadKinBodyXMLFile('tabletop_2.kinbody.xml')
env.Add(table)

manip = robot.SetActiveManipulator('right_arm')
#print(manip.GetArmIndices()) # [ 8  7  9  5  6 33 32] out of 34
RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()
robot.SetDOFValues([0.0]*34,range(34))     # 5*2+2+22


# enter one or more initial conditions for the robot configuration
X0 = [0.0]*34
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
# INPUT: Array with [x,y,z,theta] information in world coordinates for the target object and obstacles. Target object should be entered in the first column. Also, is_box Boolean indicates whether target object is box or not.
# enter obstacles location and orientation in world coordinate frame
INPUT = np.array([[0.1, 0.1, 0.0, 45*np.pi/180],  
                [0.0,-0.2, 0.0, 0*np.pi/180],   
                [0.4, 0.3, 0.0, 0*np.pi/180],
                [0.0, 0.3, 0.0, 0*np.pi/180]])
is_box = False



Theta_min = -30*np.pi/180 # choose interval Theta_min - Theta_max for wrist orientation wrt the object if it is not a box
Theta_max = 90*np.pi/180
choose = 1 # choose which one to plan


# Calculate transformation matrices for obstacles and target object
def get_transf_mat_from_pos_orient(xyz,theta):
    return np.array([[np.cos(theta),-np.sin(theta),0,xyz[0]],[np.sin(theta),np.cos(theta),0,xyz[1]],[0,0,1,xyz[2]],[0,0,0,1]])   
Tobj = []
for i in range(INPUT.shape[0]):
    xyz = INPUT[i,0:3]
    theta = INPUT[i,-1]
    Tobj.append(get_transf_mat_from_pos_orient(xyz,theta))

if is_box:
        Theta = np.array([-np.pi/2,0,np.pi/2])
        num_processes = 3
else:
    Theta = np.linspace(Theta_min,Theta_max,num=5)  # choose a few wrist orientations in the interval 0 to 90
print 'Wrist rotation angles in deg: ' , Theta*180/np.pi
Tgoal=[]
for theta in Theta:
    Trans = get_transf_mat_from_pos_orient(np.array([0,0,0]),theta) # calculate transformation matrix for varied palm orientations
    Apo_varried = np.matmul(Trans,Apo) # get varied Apo
    Tgoal.append(np.matmul(Tobj[0],Apo_varried)) # transform to world coordinate frame   

for i in range(len(Tobj)):
    if i == 0:
        target = env.ReadKinBodyXMLFile('target.kinbody.xml')
        env.Add(target)
        target.SetTransform(Tobj[i])
    else:
        kinbody = 'obs%s.kinbody.xml' % i
        exec "obs%s=env.ReadKinBodyXMLFile(kinbody)" % i
        exec "env.Add(obs%s)" % i
        exec "obs%s.SetTransform(Tobj[i])" % i

h = env.plot3(Tobj[0][0:3,3],30)
raw_input('press any key 3')


        
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

