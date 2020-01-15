from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()
import time
from openravepy import *
from openravepy.ikfast import *
import numpy as np
import os
from multiprocessing import Process, Pipe
from pdb import set_trace as bp
import pandas as pd


#def eprint(*args, **kwargs):
#    print(*args, file=sys.stderr, **kwargs)

###############################
####### Child process  ########
###############################
def child_process(conn):
    print('simulation process id:', os.getpid())
    # Receive the initialization data
    initialization_data = conn.recv()
    #print('Initialization data: ', initialization_data)
    Tobj = initialization_data[1]

    # Initialize stuff using the data from the main program, e.g. create client
    env = Environment()
    env.SetViewer('qtcoin')  #comment out if not using visualization!
    urdf_module = RaveCreateModule(env, 'urdf')
    urdf_path = "package://inmoov_description/robots/inmoov_shadow_hand_v2_1.urdf"
    srdf_path = "package://inmoov_description/srdf/inmoov_shadow_hand_v2.srdf"
    inmoov_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
    #print('robot name:', inmoov_name)
    robot = env.GetRobots()[0]
    #print(robot)
    baseT_translation = np.array([-0.3, 0.5, -1.25])
    BaseT = np.array([[1,0,0,baseT_translation[0]],
                [0,1,0,baseT_translation[1]],
                [0,0,1,baseT_translation[2]],
                [0,0,0,1]])
    robot.SetTransform(BaseT)
    
    table = env.ReadKinBodyXMLFile('tabletop_2.kinbody.xml')
    env.Add(table)
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


    manip = robot.SetActiveManipulator('right_arm')
    #RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs


    
    while True:
        command_and_args = conn.recv()   # Get command from main process
        if command_and_args[0] == "plan":    # Run one trajectory
            
            robot.SetDOFValues(command_and_args[1],range(34))   
            Tgoal = command_and_args[2]
            try:
                res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10,outputtrajobj=True)#,execute=False)
                traj = res.GetAllWaypoints2D()[:,0:-1]
                J = np.linalg.norm(traj[-1,0:7]) #fitness function is the norm of the q's at the final state.    
            except:
                traj = []
                J = np.inf          
        if command_and_args[0] == "get_sol":     # Get all the Js in the buffer
            conn.send([traj,J])
        if command_and_args[0] == "stop":       # Stop the program
            conn.close()
            break





###############################
######## Main process  ########
###############################
if __name__ == '__main__':
    
    file_path = 'input.csv'
    while not os.path.exists(file_path):
       time.sleep(1)
    if os.path.isfile(file_path):
       data = pd.read_csv(file_path,sep=',')
    else:
        raise ValueError("%s isn't a file!" % file_path)


    num_processes = 5# number of parallel processes to use

    print('main program process id:', os.getpid())
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


    # Calculate transformation matrices for obstacles and target object
    def get_transf_mat_from_pos_orient(xyz,theta):
        return np.array([[np.cos(theta),-np.sin(theta),0,xyz[0]],[np.sin(theta),np.cos(theta),0,xyz[1]],[0,0,1,xyz[2]],[0,0,0,1]])   
    Tobj = []
    for i in range(INPUT.shape[0]):
        xyz = INPUT[i,0:3]
        theta = INPUT[i,-1]
        Tobj.append(get_transf_mat_from_pos_orient(xyz,theta))
    
    # Calculate transformation matrices for wrist target locations
    if is_box:
        Theta = np.array([-np.pi/2,0,np.pi/2])
        num_processes = 3
    else:
        Theta = np.linspace(Theta_min,Theta_max,num=num_processes)  # choose a few wrist orientations in the interval 0 to 90
    print 'Wrist rotation angles in deg: ' , Theta*180/np.pi
    Tgoal=[]
    for theta in Theta:
        Trans = get_transf_mat_from_pos_orient(np.array([0,0,0]),theta) # calculate transformation matrix for varied palm orientations
        Apo_varried = np.matmul(Trans,Apo) # get varied Apo
        Tgoal.append(np.matmul(Tobj[0],Apo_varried)) # transform to world coordinate frame   

    # Begin Processes
    #bp()
    processes = []
    for i in range(num_processes):
        # Create a pair of pipes that can communicate with each other through .send() and .recv()
        parent_conn, child_conn = Pipe()
        # Create the sub-process and assign the pipe to it
        p = Process(target=child_process, args=(child_conn,))
        processes.append([p, parent_conn])
        # Start the process
        p.start()
        # Send the initial arguments for initialization
        parent_conn.send([i,Tobj])

    

    start_time = time.time()
    for i in range(num_processes):
            processes[i][1].send(["plan", X0, Tgoal[i]])    # Ask process i to plan trajectory from initial condition X0 to final condition Tgoal

    Traj = []
    Cost = []
    for i in range(num_processes): 
            processes[i][1].send(["get_sol"])     # Get the planned trajectory
            Traj_i, Cost_i = processes[i][1].recv()  
            Traj.append(Traj_i)
            Cost.append(Cost_i)

    end_time = time.time()
    print("Duration: %.2f sec" % (end_time-start_time))

    bp() 
    for i in range(num_processes):
        Js_i = processes[i][1].send(["stop"])
        processes[i][0].join()        

    OptTraj = Traj[np.argmin(Cost)]

    df = pd.DataFrame({'OptTraj':OptTraj})
    df.to_csv('OR_REACH.csv')   