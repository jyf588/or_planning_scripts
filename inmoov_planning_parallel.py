from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()
import time
from openravepy import *
from openravepy.ikfast import *
import numpy as np
import os
from multiprocessing import Process, Pipe
from pdb import set_trace as bp



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
        
    # Initialize stuff using the data from the main program, e.g. create client
    env = Environment()
    #env.SetViewer('qtcoin')
    urdf_module = RaveCreateModule(env, 'urdf')
    urdf_path = "package://inmoov_description/robots/inmoov_shadow_hand_v2.urdf"
    srdf_path = "package://inmoov_description/srdf/inmoov_shadow_hand_v2.srdf"
    inmoov_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
    #print('robot name:', inmoov_name)
    robot = env.GetRobots()[0]
    #print(robot)
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
                res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10,outputtrajobj=True,execute=False)
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
    num_processes = 5# number of parallel processes to use

    print('main program process id:', os.getpid())
    
    # enter one or more initial conditions for the robot configuration
    X0 = [0.0]*34

    
    ########## enter object location and orientation in world coordinate frame
    obj_xyz = np.array([0.10, 0.10, 0])
    obj_theta = 45*np.pi/180 #rotation is assumed to be around z-axis
    is_box = True
    Theta_min = 0 # choose interval Theta_min - Theta_max for wrist orientation wrt the object if it is not a box
    Theta_max = 2*np.pi/3


    
    # calculate transformation matrix
    RotMat_w_o = np.array([[np.cos(obj_theta),-np.sin(obj_theta),0,obj_xyz[0]],[np.sin(obj_theta),np.cos(obj_theta),0,obj_xyz[1]],[0,0,1,obj_xyz[2]],[0,0,0,1]]) # calculate rotation matrix
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
         num_processes = 3
    else:
        Theta = np.linspace(0,2*np.pi/3,num=num_processes)  # choose a few wrist orientations in the interval 0 to 90
    print 'Wrist rotation angles in deg: ' , Theta*180/np.pi
    Tgoal=[]
    for theta in Theta:
        RotMat = np.array([[np.cos(theta),-np.sin(theta),0,0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]]) # calculate rotation matrix
        Apo_varried = np.matmul(RotMat,Apo) # get varied Apo
        Tgoal.append(np.matmul(RotMat_w_o,Apo_varried)) # transform to world coordinate frame   
        
    #---------- Setup other stuff, client for real world, etc
    # Setup and store all processes
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
        parent_conn.send([i, 'something else'])

    

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


    for i in range(num_processes):
        Js_i = processes[i][1].send(["stop"])
        processes[i][0].join()        

    bp()    