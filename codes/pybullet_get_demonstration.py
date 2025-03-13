import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import math
import openpyxl
import datetime

# ---------Initialize Simulation Environment--------
# set pybullet
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1)
p.setAdditionalSearchPath(pd.getDataPath())
# load object models
tableUid = p.loadURDF("table/table.urdf", [0.3, -0.6, -0.5], [-0.5, -0.5, -0.5, 0.5], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
objectUid = p.loadURDF("random_urdfs/000/000.urdf", [0.5, 0.2, -0.5], [-0.5, -0.5, -0.5, 0.5], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
# load robotic arm models: Emika panda
# note: unlike above models of objects, many parameters and constraints of the robot model need to be defined for proper control
useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7
ll = [-7]*pandaNumDofs
ul = [7]*pandaNumDofs
jr = [7]*pandaNumDofs
jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions
orn=[-0.707107, 0.0, 0.0, 0.707107]
offset = [0, 0, 0]
pandaUid = p.loadURDF("franka_panda/panda.urdf", np.array([0,0,0])+offset, orn, useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
index = 0
for j in range(p.getNumJoints(pandaUid)):
    p.changeDynamics(pandaUid, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(pandaUid, j)
    jointName = info[1]
    jointType = info[2]
    if (jointType == p.JOINT_PRISMATIC):
        p.resetJointState(pandaUid, j, jointPositions[index])
        index = index + 1
    if (jointType == p.JOINT_REVOLUTE):
        p.resetJointState(pandaUid, j, jointPositions[index])
        index = index + 1
# Set gravity and camera position
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=270, cameraPitch=-45, cameraTargetPosition=[0.5, 0, -0.5])


# -----------Start Simulation-----------
timeStep = 1. / 60.
p.setTimeStep(timeStep)
p.setGravity(0, -9.8, 0)
t = 0
pos_control = [0.2 * math.sin(1.5 * 0), 0.3, -0.6 + 0.1 * math.cos(1.5 * 0)] # initial control pos
control_step = 0.008
finger = 1 # initialize the grasper in open state
sampling_cnt = 0
data_list = []
simulate_phase = 0

while (1):
    p.stepSimulation()
    time.sleep(timeStep)

    # Read keyboard input for robotic arm control
    # Press 'q' to start the demo and 'e' to stop
    keys = p.getKeyboardEvents()
    if ord("q") in keys:
        print("[-----Get demo: begin------]")
        simulate_phase = 1
    if ord("e") in keys:
        break
    # 'W' and 'S' to move the robot go forward and backward
    if ord("w") in keys:
        pos_control[0] = pos_control[0] - control_step
    if ord("s") in keys:
        pos_control[0] = pos_control[0] + control_step
    # 'A' and 'D' to move the robot move left and right
    if ord("a") in keys:
        pos_control[2] = pos_control[2] + control_step
    if ord("d") in keys:
        pos_control[2] = pos_control[2] - control_step
    # 'Z' and 'X' to set the robot's  height
    if ord("z") in keys:
        pos_control[1] = pos_control[1] + control_step
    if ord("x") in keys:
        pos_control[1] = pos_control[1] - control_step
    # 'space' to grasp the object
    if ord(" ") in keys:
        finger = 0

    # calculate control command
    pos = [offset[0]+pos_control[0], offset[1]+pos_control[1], offset[2]+pos_control[2]]
    orn = p.getQuaternionFromEuler([math.pi / 2., 0., 0.])
    jointPoses = p.calculateInverseKinematics(pandaUid, pandaEndEffectorIndex, pos, orn, ll, ul,
                                                               jr, rp, maxNumIterations=5)

    # sampling the states in 10hz
    if simulate_phase == 1:
        sampling_cnt = sampling_cnt + 1
    if sampling_cnt == 6:  #渲染6帧采样一次
        sampling_cnt = 0 # 计数器清零

        # get the pos of robot's end effector
        end_state = p.getLinkState(pandaUid, pandaEndEffectorIndex)
        end_pos = end_state[0]
        # get the pos of target object
        object_pos, _ = p.getBasePositionAndOrientation(objectUid)
        data_list.append([*end_pos, *object_pos, finger])

    # send control command
    for i in range(pandaNumDofs):
        p.setJointMotorControl2(pandaUid, i, p.POSITION_CONTROL, jointPoses[i], force=5 * 240.)
    p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, finger)
    p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, finger)


# ------ Data Store after Quit the Simulation -------------
p.disconnect()
print("[-----Get demo: over-------]")
wb = openpyxl.Workbook()  # initial excel
ws = wb.active
# writing headers
ws.append(["End_pos_x", "End_pos_y", "End_pos_z",
           "Object_pos_x", "Object_pos_y", "Object_pos_z", "finger"])
# input excel
for data in data_list:
    ws.append(data)
# save .Excel
dt = datetime.datetime.now()
dt_str = dt.strftime("%Y-%m-%d_%H-%M-%S")
excel_filename = "demo" + dt_str + ".xlsx"
wb.save(excel_filename)
print("[---------Demo saved--------]")