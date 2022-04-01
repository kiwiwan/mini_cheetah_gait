import numpy as np
from pydrake.all import (
        Parser, RigidTransform,
        Box, CoulombFriction
)

from typing import NamedTuple
from abc import abstractmethod

class Robot():
    def __init__(self, plant, model_filename):
        self.plant = plant
        parser = Parser(self.plant)
        self.model = parser.AddModelFromFile(model_filename)
        self.plant.Finalize()
        self.nq = self.plant.num_positions()
        self.nv = self.plant.num_velocities()

    @abstractmethod
    def get_contact_frames(self):
        pass

    @abstractmethod
    def set_home(self, plant, context):
        pass


    def get_total_mass(self, context):
        return sum(self.plant.get_body(index).get_mass(context) for index in self.plant.GetBodyIndices(self.model))

    def get_num_contacts(self):
        return len(self.get_contact_frames())



class MiniCheetah(Robot):
    class JointLimit(NamedTuple):
        effort: float
        lower: float
        upper: float
        velocity: float

    JOINT_LIMITS = {
            "torso_to_abduct_fl_j" : JointLimit(18, -1.6, 1.6, 40),
            "abduct_fl_to_thigh_fl_j" : JointLimit(18, -2.6, 2.6, 40),
            "thigh_fl_to_knee_fl_j" : JointLimit(26, -2.6, 2.6, 26),

            "torso_to_abduct_fr_j" : JointLimit(18, -1.6, 1.6, 40),
            "abduct_fr_to_thigh_fr_j" : JointLimit(18, -2.6, 2.6, 40),
            "thigh_fr_to_knee_fr_j" : JointLimit(26, -2.6, 2.6, 26),

            "torso_to_abduct_hl_j" : JointLimit(18, -1.6, 1.6, 40),
            "abduct_hl_to_thigh_hl_j" : JointLimit(18, -2.6, 2.6, 40),
            "thigh_hl_to_knee_hl_j" : JointLimit(26, -2.6, 2.6, 26),

            "torso_to_abduct_hr_j" : JointLimit(18, -1.6, 1.6, 40),
            "abduct_hr_to_thigh_hr_j" : JointLimit(18, -2.6, 2.6, 40),
            "thigh_hr_to_knee_hr_j" : JointLimit(26, -2.6, 2.6, 26),
    }

    CONTACTS_PER_FRAME = {
            "LF_FOOT": np.array([
                [0, 0, 0,] 
            ]).T,
            "RF_FOOT": np.array([
                [0, 0, 0,] 
            ]).T,
            "LH_FOOT": np.array([
                [0, 0, 0,] 
            ]).T,
            "RH_FOOT": np.array([
                [0, 0, 0,] 
            ]).T
    }

    NUM_ACTUATED_DOF = 12

    def __init__(self, plant, add_ground=True):
        if add_ground:
            color = np.array([.9, .9, .9, 1.0])

            box = Box(30., 30., 1.)
            X_WBox = RigidTransform([0, 0, -0.5-0.0175])

            plant.RegisterVisualGeometry(plant.world_body(), X_WBox, box,
                    "GroundVisuaGeometry", color)

            ground_friction = CoulombFriction(1.0, 1.0)
            plant.RegisterCollisionGeometry(plant.world_body(), X_WBox, box,
                    "GroundCollisionGeometry", ground_friction)
            plant.set_penetration_allowance(1.0e-3)
            plant.set_stiction_tolerance(1.0e-3)

        super().__init__(plant, "robots/mini_cheetah/mini_cheetah_mesh.urdf")


    def get_contact_frames(self):
        return [
            self.plant.GetFrameByName('LF_FOOT'),
            self.plant.GetFrameByName('RF_FOOT'),
            self.plant.GetFrameByName('LH_FOOT'),
            self.plant.GetFrameByName('RH_FOOT')]

    def get_contact_frame_names(self):
        return [
            'LF_FOOT',
            'RF_FOOT',
            'LH_FOOT',
            'RH_FOOT']
            
    def get_body_name(self):
        return "body"

    def set_home(self, plant, context):
        hip_roll = .1;
        # hip_pitch = 1;
        hip_pitch = 0.5;
        knee = 1.55;
        plant.GetJointByName("torso_to_abduct_fr_j").set_angle(context, -hip_roll)
        plant.GetJointByName("abduct_fr_to_thigh_fr_j").set_angle(context, -hip_pitch)
        plant.GetJointByName("thigh_fr_to_knee_fr_j").set_angle(context, knee)
        plant.GetJointByName("torso_to_abduct_fl_j").set_angle(context, hip_roll)
        plant.GetJointByName("abduct_fl_to_thigh_fl_j").set_angle(context, -hip_pitch)
        plant.GetJointByName("thigh_fl_to_knee_fl_j").set_angle(context, knee)
        plant.GetJointByName("torso_to_abduct_hr_j").set_angle(context, -hip_roll)
        plant.GetJointByName("abduct_hr_to_thigh_hr_j").set_angle(context, -hip_pitch)
        plant.GetJointByName("thigh_hr_to_knee_hr_j").set_angle(context, knee)
        plant.GetJointByName("torso_to_abduct_hl_j").set_angle(context, hip_roll)
        plant.GetJointByName("abduct_hl_to_thigh_hl_j").set_angle(context, -hip_pitch)
        plant.GetJointByName("thigh_hl_to_knee_hl_j").set_angle(context, knee)
        plant.SetFreeBodyPose(context, plant.GetBodyByName("body"), RigidTransform([0, 0, 0.27]))  #0.24984  +0.0175  0.270375

    

def getJointValues(plant, joint_names, context):
    ret = []
    for name in joint_names:
        ret.append(plant.GetJointByName(name).get_angle(context))
    return ret


def setJointValues(plant, joint_values, context):
    for i in range(len(joint_values)):
        plant.GetJointByIndex(i).set_angle(context, joint_values[i])            

