from pydrake.all import (
        RotationMatrix, Quaternion
)
from Gait import Gait
from FootSwingTrajectory import FootSwingTrajectory
import numpy as np

class StateEstimator():
    def __init__(self):
        pass


    def setContactPhase(self, contactPhase):
        self.contactPhase = contactPhase

    def update(self, position, vBody, orientation, omegaBody, rBody, rpy, omegaWorld, vWorld, aBody, aWorld, footInHip, hipLocation):
        #self.contactEstimate = contactEstimate
        self.position = position
        self.vBody = vBody
        self.orientation = orientation
        self.omegaBody = omegaBody
        self.rBody = rBody
        self.rpy = rpy
        self.omegaWorld = omegaWorld
        self.vWorld = vWorld
        self.aBody = aBody
        self.aWorld = aWorld

        self.footInHip = footInHip
        self.hipLocation = hipLocation


 

class GaitCommand():
    def __init__(self, iterationsBetweenMPC, dt):
        self.footSwingTrajectories = [FootSwingTrajectory(), FootSwingTrajectory(), FootSwingTrajectory(), FootSwingTrajectory()]

        self._x_vel_des = 0
        self._y_vel_des = 0
        self.iterationsBetweenMPC = iterationsBetweenMPC
        self.horizonLength = 10
        self.dt = dt

        self.dtMPC = self.dt * self.iterationsBetweenMPC
        self.iterationCounter = 0

        self.rpy_comp = np.zeros(3)
        self.rpy_int = np.zeros(3)
    

        self.cmpc_bonus_swing = 0

        self.firstSwing = np.zeros(4)
        for i in range(4): 
            self.firstSwing[i] = True
        self.firstRun = True

        self.pFoot = np.zeros((4,3))
        self.pFoot_des = np.zeros((4,3))
        self.vFoot_des = np.zeros((4,3))
        self.aFoot_des = np.zeros((4,3))

        self.stand_traj = np.zeros(6)
        self.world_position_desired = np.zeros(3)
        self.swingTimes = np.zeros(4)
        self.swingTimeRemaining = np.zeros(4)

        self.current_gait = 0
        self.x_vel_cmd = 0
        self.y_vel_cmd = 0
        self._yaw_turn_rate = 0

        self.trotting = Gait(self.horizonLength, np.array([0,5,5,0]), np.array([5,5,5,5]), "Trotting")
        self.bounding = Gait(self.horizonLength, np.array([5,5,0,0]), np.array([4,4,4,4]), "Bounding")
        self.pronking = Gait(self.horizonLength, np.array([0,0,0,0]), np.array([4,4,4,4]), "Pronking")
        self.jumping = Gait(self.horizonLength, np.array([0,0,0,0]), np.array([2,2,2,2]), "Jumping")
        self.galloping = Gait(self.horizonLength, np.array([0,2,7,9]), np.array([4,4,4,4]), "Galloping")
        self.standing = Gait(self.horizonLength, np.array([0,0,0,0]), np.array([10,10,10,10]), "Standing")
        self.trotRunning = Gait(self.horizonLength, np.array([0,5,5,0]), np.array([4,4,4,4]), "Trot Running")
        self.walking = Gait(self.horizonLength, np.array([0,3,5,8]), np.array([5,5,5,5]), "Walking")
        self.walking2 = Gait(self.horizonLength, np.array([0,5,5,0]), np.array([7,7,7,7]), "Walking2")
        self.pacing = Gait(self.horizonLength, np.array([5,0,5,0]), np.array([5,5,5,5]), "Pacing")
        
        self.pBody_des = np.zeros(3)
        self.vBody_des = np.zeros(3)
        self.aBody_des = np.zeros(3)
        self.pBody_RPY_des = np.zeros(3)
        self.vBody_Ori_des = np.zeros(3)

    def setupCommand(self):#, desiredCommand
        self._body_height = 0.29

        # self._yaw_turn_rate = 0
        # x_vel_cmd = 0
        # y_vel_cmd = 0

        if self._stateEstimator.vWorld[0] <= 2.:
            self.x_vel_cmd += 0.005

        filter = 0.1
        self._x_vel_des = self._x_vel_des*(1-filter) + self.x_vel_cmd*filter
        self._y_vel_des = self._y_vel_des*(1-filter) + self.y_vel_cmd*filter

        self._yaw_des = self._stateEstimator.rpy[2] + self.dt * self._yaw_turn_rate
        self._roll_des = 0.
        self._pitch_des = 0.

    def run(self, gaitNumber, stateEstimator):
        self._stateEstimator = stateEstimator
        self.setupCommand()
        seResult = self._stateEstimator
        if (gaitNumber == 9 and self.current_gait != 9) or self.firstRun:
            self.stand_traj[0] = seResult.position[0]
            self.stand_traj[1] = seResult.position[1]
            self.stand_traj[2] = 0.29
            self.stand_traj[3] = 0
            self.stand_traj[4] = 0
            self.stand_traj[5] = seResult.rpy[2]
            self.world_position_desired[0] = self.stand_traj[0]
            self.world_position_desired[1] = self.stand_traj[1]

        self.gait = self.trotting
        if gaitNumber == 1:
            self.gait = self.trotting
        elif gaitNumber == 2:
            self.gait = self.trotRunning
        elif gaitNumber == 3:
            self.gait = self.galloping
        elif gaitNumber == 4:
            self.gait = self.bounding
        elif gaitNumber == 5:
            self.gait = self.pronking
        elif gaitNumber == 6:
            self.gait = self.pacing
        elif gaitNumber == 7:
            self.gait = self.walking
        elif gaitNumber == 8:
            self.gait = self.walking2
        elif gaitNumber == 9:
            self.gait = self.standing

        self.current_gait = gaitNumber

        self.gait.setIterations(self.iterationsBetweenMPC, self.iterationCounter)

        if self._body_height < 0.02:
            self._body_height = 0.29

        v_des_robot = np.array([self._x_vel_des, self._y_vel_des, 0])
        v_des_world = seResult.rBody.T.dot(v_des_robot)
        v_robot = seResult.vWorld.copy()

        if abs(v_robot[0]) > .2:
            self.rpy_int[1] += self.dt*(self._pitch_des - seResult.rpy[1])/v_robot[0]
        if abs(v_robot[1]) > 0.1:
            self.rpy_int[0] += self.dt*(self._roll_des - seResult.rpy[0])/v_robot[1]

        self.rpy_int[0] = min(max(self.rpy_int[0], -.25), .25)
        self.rpy_int[1] = min(max(self.rpy_int[1], -.25), .25)
        self.rpy_comp[1] = v_robot[0] * self.rpy_int[1]
        self.rpy_comp[0] = v_robot[1] * self.rpy_int[0] * (1 if gaitNumber!=5 else 0)  #turn off for pronking

        
        for i in range(4): 
            self.pFoot[i] = seResult.position + seResult.rBody.T.dot(seResult.hipLocation[i] + seResult.footInHip[i])


        if self.gait != self.standing:
            self.world_position_desired += self.dt * np.array([v_des_world[0], v_des_world[1], 0])

        if self.firstRun:
            self.world_position_desired[0] = seResult.position[0]
            self.world_position_desired[1] = seResult.position[1]
            self.world_position_desired[2] = seResult.rpy[2]

            for i in range(4):
                self.footSwingTrajectories[i].setHeight(0.05)
                self.footSwingTrajectories[i].setInitialPosition(self.pFoot[i].copy())
                self.footSwingTrajectories[i].setFinalPosition(self.pFoot[i].copy())


            self.firstRun = False



        for i in range(4):
            self.swingTimes[i] = self.gait.getCurrentSwingTime(self.dtMPC, i)

        side_sign = [-1, 1, -1, 1] 
        interleave_y = [-0.08, 0.08, 0.02, -0.02]
        interleave_gain = -0.2
        v_abs = abs(v_des_robot[0])

        for i in range(4):   
            if self.firstSwing[i]:
                self.swingTimeRemaining[i] = self.swingTimes[i]
            else:
                self.swingTimeRemaining[i] -= self.dt
            self.footSwingTrajectories[i].setHeight(.06)
            offset = np.array([0, side_sign[i] * .065, 0])
            pRobotFrame = (seResult.hipLocation[i] + offset)
            pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain

            stance_time = self.gait.getCurrentStanceTime(self.dtMPC, i)
            pYawCorrected = (RotationMatrix.MakeZRotation(-self._yaw_turn_rate* stance_time / 2).matrix()).dot(pRobotFrame) 

            des_vel = np.array([self._x_vel_des, self._y_vel_des, 0])
            Pf = seResult.position + seResult.rBody.T.dot(pYawCorrected + des_vel * self.swingTimeRemaining[i])
            p_rel_max = 0.3
            pfx_rel = seResult.vWorld[0] * (.5 + self.cmpc_bonus_swing) * stance_time +.03*(seResult.vWorld[0]- v_des_world[0]) + (0.5*seResult.position[2]/9.81) * (seResult.vWorld[1]*self._yaw_turn_rate)

            pfy_rel = seResult.vWorld[1] * .5 * stance_time * self.dtMPC + .03*(seResult.vWorld[1]-v_des_world[1]) + (0.5*seResult.position[2]/9.81) * (-seResult.vWorld[0]*self._yaw_turn_rate)

            pfx_rel = min(max(pfx_rel, -p_rel_max), p_rel_max)
            pfy_rel = min(max(pfy_rel, -p_rel_max), p_rel_max)
            Pf[0] +=  pfx_rel
            Pf[1] +=  pfy_rel
            Pf[2] = -0.003
            self.footSwingTrajectories[i].setFinalPosition(Pf.copy())

        
        # calc gait
        self.iterationCounter += 1

        se_contactState = np.zeros(4)

        contactStates = self.gait.getContactState()
        swingStates = self.gait.getSwingState()
        mpcTable = self.gait.getMpcTable()

        for foot in range(4):
            contactState = contactStates[foot]
            swingState = swingStates[foot]
            if swingState > 0:  #foot is in swing
                if self.firstSwing[foot]:
                    self.firstSwing[foot] = False
                    self.footSwingTrajectories[foot].setInitialPosition(self.pFoot[foot].copy())

                
                self.footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, self.swingTimes[foot])
                pDesFootWorld = self.footSwingTrajectories[foot].getPosition()
                vDesFootWorld = self.footSwingTrajectories[foot].getVelocity()
                pDesLeg = seResult.rBody.dot(pDesFootWorld - seResult.position) - seResult.hipLocation[foot]
                vDesLeg = seResult.rBody.dot(vDesFootWorld - seResult.vWorld)

                self.pFoot_des[foot] = pDesFootWorld
                self.vFoot_des[foot] = vDesFootWorld
                self.aFoot_des[foot] = self.footSwingTrajectories[foot].getAcceleration()
            else:
                self.firstSwing[foot] = True

                # pDesFootWorld = self.footSwingTrajectories[foot].getPosition()
                # vDesFootWorld = self.footSwingTrajectories[foot].getVelocity()
                pDesFootWorld = self.pFoot[foot].copy()
                vDesFootWorld = np.zeros(3)
                pDesLeg = seResult.rBody.dot(pDesFootWorld - seResult.position) - seResult.hipLocation[foot]
                vDesLeg = seResult.rBody.dot(vDesFootWorld - seResult.vWorld)

                self.pFoot_des[foot] = pDesFootWorld
                self.vFoot_des[foot] = vDesFootWorld
                self.aFoot_des[foot] = self.footSwingTrajectories[foot].getAcceleration()

                se_contactState[foot] = contactState.copy()


        self._stateEstimator.setContactPhase(se_contactState)
        # print("foot1 p0:",self.footSwingTrajectories[1]._p0)
        # print("###########")


        # Update For WBC
        self.pBody_des[0] = self.world_position_desired[0]
        self.pBody_des[1] = self.world_position_desired[1]
        self.pBody_des[2] = self._body_height

        self.vBody_des[0] = v_des_world[0]
        self.vBody_des[1] = v_des_world[1]
        self.vBody_des[2] = 0.

        self.aBody_des = np.zeros(3)

        self.pBody_RPY_des[0] = 0.
        self.pBody_RPY_des[1] = 0.
        self.pBody_RPY_des[2] = self._yaw_des

        self.vBody_Ori_des[0] = 0.
        self.vBody_Ori_des[1] = 0.
        self.vBody_Ori_des[2] = self._yaw_turn_rate


        contact_state = self.gait.getContactState().copy()
        # END of for WBC
  

    def initialize(self):
        for i in range(4): 
            self.firstSwing[i] = True
        self.firstRun = True

    
