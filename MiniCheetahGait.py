import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder, 
    RotationMatrix, PiecewisePolynomial, JacobianWrtVariable,
    Quaternion, RollPitchYaw, ConnectMeshcatVisualizer, InverseKinematics, Solve,
    MeshcatVisualizer
    
)
from MiniCheetah import MiniCheetah

from GaitCommand import GaitCommand
from GaitCommand import StateEstimator

from functools import partial
import time

from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
proc, zmq_url, web_url = start_zmq_server_as_subprocess()

def MiniCheetahGait(robot_ctor):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
    robot = robot_ctor(plant)
    # visualizer = ConnectMeshcatVisualizer(builder,
    #     scene_graph=scene_graph,
    #     zmq_url=zmq_url)
    #need modify draw_period to make record time normal
    visualizer = builder.AddSystem(MeshcatVisualizer(scene_graph=scene_graph, draw_period=0.002, zmq_url=zmq_url))
    builder.Connect(scene_graph.get_query_output_port(),
                    visualizer.get_geometry_query_input_port())
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    robot.set_home(plant, plant_context)
    visualizer.load()
    diagram.Publish(context)


    stateEstimator = StateEstimator()
    gait_cmd = GaitCommand(27 / (1000. * 0.002),0.002)


    while True:
        str = input("Enter f:")
        if str == 'f':
            strnum = input("Enter a number(1-9):")
            if strnum == '1':
                gaitNumber = 1
            elif strnum == '2':
                gaitNumber = 2
            elif strnum == '3':
                gaitNumber = 3
            elif strnum == '4':
                gaitNumber = 4
            elif strnum == '5':
                gaitNumber = 5
            elif strnum == '6':
                gaitNumber = 6
            elif strnum == '7':
                gaitNumber = 7
            elif strnum == '8':
                gaitNumber = 8
            elif strnum == '9':
                gaitNumber = 9
            else:
                gaitNumber = 1

            # visualizer.start_recording()
            q_des_list = []
            v_des_list = []
            for count in range(1000):
                start =time.clock()
                #get robot state
                q_measure = plant.GetPositions(plant_context)
                v_measure = plant.GetVelocities(plant_context)
                body_frame = plant.GetFrameByName(robot.get_body_name())
                contact_frame = robot.get_contact_frames()
                # hip_frame_name = ['LF_HIP', 'RF_HIP', 'LH_HIP', 'RH_HIP']
                hip_frame_name = ['RF_HIP', 'LF_HIP', 'RH_HIP', 'LH_HIP']
                foot_frame_name = ['RF_FOOT', 'LF_FOOT', 'RH_FOOT', 'LH_FOOT']


                position = q_measure[4:7]
                orientation = q_measure[:4]
                vWorld = v_measure[3:6]
                omegaWorld = v_measure[:3]
                aWorld = np.zeros(3)
                rBody = RotationMatrix(Quaternion(orientation)).inverse().matrix()
                rpy = RollPitchYaw(RotationMatrix(Quaternion(orientation)).inverse()).vector()
                vBody = rBody.T.dot(vWorld)
                omegaBody = rBody.T.dot(omegaWorld)
                aBody = rBody.T.dot(aWorld)

                footInHip = np.zeros((4,3))
                hipLocation = np.zeros((4,3))
                for i in range(4): 
                    X_WF = plant.CalcRelativeTransform(plant_context, body_frame, plant.GetFrameByName(hip_frame_name[i]))
                    X_WF1 = plant.CalcRelativeTransform(plant_context, plant.GetFrameByName(hip_frame_name[i]), plant.GetFrameByName(foot_frame_name[i]))
                    hipLocation[i] = X_WF.translation()
                    footInHip[i] = X_WF1.translation()

 
                #run gait code
                stateEstimator.update(position, vBody, orientation, omegaBody, rBody, rpy, omegaWorld, vWorld, aBody, aWorld, footInHip, hipLocation)
                # gaitNumber = 1
                gait_cmd.run(gaitNumber, stateEstimator)

                #get result
                pBody_des = gait_cmd.pBody_des
                vBody_des = gait_cmd.vBody_des
                aBody_des = gait_cmd.aBody_des
                pBody_RPY_des = gait_cmd.pBody_RPY_des
                vBody_Ori_des = gait_cmd.vBody_Ori_des

                pFoot_des = gait_cmd.pFoot_des
                vFoot_des = gait_cmd.vFoot_des
                aFoot_des = gait_cmd.aFoot_des

                #set robot state
                q_des = np.array(q_measure)
                v_des = np.array(v_measure)
                q_des[4:7] = pBody_des
                q_des[:4] = Quaternion(RotationMatrix(RollPitchYaw(pBody_RPY_des)).matrix()).wxyz()
                v_des[3:6] = vBody_des
                v_des[:3] = vBody_Ori_des
                q_des[7:] = q_measure[7:]



                #solve leg ik,set plant position
                ik = InverseKinematics(plant, plant_context)
                ik.AddPositionConstraint(body_frame, [0, 0, 0], plant.world_frame(), q_des[4:7], q_des[4:7])
                ik.AddOrientationConstraint(body_frame, RotationMatrix(), plant.world_frame(), RotationMatrix(Quaternion(q_des[:4])), 0)
                for i in range(4):
                    ik.AddPositionConstraint(plant.GetFrameByName(foot_frame_name[i]), [0, 0, 0], plant.world_frame(), pFoot_des[i], pFoot_des[i])
                prog = ik.get_mutable_prog()
                q = ik.q()
                prog.AddQuadraticErrorCost(np.identity(len(q)), q_des, q)
                prog.SetInitialGuess(q, q_des)
                result = Solve(ik.prog())
                if not result.is_success():
                    print("IK failed!")
                    # print(count)
                q_des = result.GetSolution(q)
                plant.SetPositions(plant_context, q_des)
                

                
                #get joint vel_des,and set plant velocity
                v_joint_des = np.zeros(12)
                for i in range(4):
                    Jv_WF = plant.CalcJacobianTranslationalVelocity(
                    plant_context, JacobianWrtVariable.kV,
                    plant.GetFrameByName(foot_frame_name[i]), [0, 0, 0], plant.world_frame(), plant.world_frame())

                    Jv_joint_WF = Jv_WF[:,6:]
                    v_joint_des += (np.linalg.inv(Jv_joint_WF.T.dot(Jv_joint_WF)-0.05*np.identity(12))).dot(Jv_joint_WF.T).dot(vFoot_des[i]) 
                
                v_des[6:18] = v_joint_des
                plant.SetVelocities(plant_context, v_des)
                q_des_list.append(q_des)
                v_des_list.append(v_des)

                diagram.Publish(context)

                # end4=time.clock()
                # print('Running time4: %s Seconds'%(end4-end3))
                # print('draw_period: %s '%(visualizer.draw_period))


            # record visualization
            visualizer.start_recording()
            for i in range(len(q_des_list)):
                plant.SetPositions(plant_context, q_des_list[i])
                plant.SetVelocities(plant_context, v_des_list[i])
                diagram.Publish(context)
            visualizer.stop_recording()
            visualizer.publish_recording()
        elif str == 'q':
            exit()

        # time.sleep(1)
    

    # print(RotationMatrix.MakeZRotation(1.57))
    # print(vBody)


          
MiniCheetahGait(MiniCheetah)

while True:
    time.sleep(2)

