'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity, sin, cos, dot
import numpy as np

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        # we don't have the wrist & hand joints (Bodytype H21)
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
                       }
        
        # wow, reading data sheets ...
        # found all values here: http://doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links
        self.offsets = {# Head
                        'HeadYaw': [0, 0, 126.5],
                        'HeadPitch': [0, 0, 0],

                        # LArm
                        'LShoulderPitch': [0, 98, 100],
                        'LShoulderRoll': [0, 0, 0],
                        'LElbowYaw': [105, 15, 0],
                        'LElbowRoll': [0, 0, 0],

                        # LLeg
                        'LHipYawPitch': [0, 50, -85],
                        'LHipRoll': [0, 0, 0],
                        'LHipPitch': [0, 0, 0],
                        'LKneePitch': [0, 0, -100],
                        'LAnklePitch': [0, 0, -102.9],
                        'LAnkleRoll': [0, 0, 0],

                        # RLeg
                        'RHipYawPitch': [0, -50, -85],
                        'RHipRoll': [0, 0, 0],
                        'RHipPitch': [0, 0, 0],
                        'RKneePitch': [0, 0, -100],
                        'RAnklePitch': [0, 0, -102.9],
                        'RAnkleRoll': [0, 0, 0],

                        # RArm
                        'RShoulderPitch': [0, -98, 100],
                        'RShoulderRoll': [0, 0, 0],
                        'RElbowYaw': [105, -15, 0],
                        'RElbowRoll': [0, 0, 0],
                        }
        
        # destinguish between types of transformation 
        # group them for later
        
        # all yaws
        #self.yaws=['HeadYaw','LElbowRoll', 'RElbowRoll', 'LShoulderRoll', 'RShoulderRoll']
        self.yaws = ['HeadYaw','LElbowYaw', 'RElbowYaw']
        # all rolls
        #self.rolls=['LHipRoll', 'LAnkleRoll', 'RHipRoll', 'RAnkleRoll', 'LElbowYaw', 'RElbowYaw']
        self.rolls = ['LHipRoll', 'LAnkleRoll', 'RHipRoll', 'RAnkleRoll', 'LElbowRoll', 'RElbowRoll', 'LShoulderRoll', 'RShoulderRoll']
        # all pitches (pitchs?)
        self.pitches = ['HeadPitch', 'LShoulderPitch', 'RShoulderPitch','LHipPitch', 'LKneePitch', 'LAnklePitch', 'RHipPitch', 'RKneePitch', 'RAnklePitch']
        # all yaw-pitches
        self.yaw_pitches = ['LHipYawPitch', 'RHipYawPitch']


    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        
        print("Current joint: ", joint_name)
        
        j_cos = cos(joint_angle)
        j_sin = sin(joint_angle)

        # standard transformations (rotations only)
        # rotation around x-axis
        Rx = [[      1,      0,      0,      0],
              [      0,  j_cos, -j_sin,      0],
              [      0,  j_sin,  j_cos,      0],
              [      0,      0,      0,      1]]

        # rotation around y-axis
        Ry = [[  j_cos,      0,  j_sin,      0],
              [      0,      1,      0,      0],
              [ -j_sin,      0,  j_cos,      0],
              [      0,      0,      0,      1]]

        # rotation around z-axis
        Rz = [[  j_cos,  j_sin,      0,      0],
              [ -j_sin,  j_cos,      0,      0],
              [      0,      0,      1,      0],
              [      0,      0,      0,      1]]
        
        if joint_name in self.pitches:
            T = Ry

        elif joint_name in self.yaws:
            T = Rz

        elif joint_name in self.rolls:
            T = Rx

        elif joint_name in self.yaw_pitches:
            # combine yaw and pitch
            T = dot(Rz, Ry)
        else:
            print('ERROR: Unknown joint name "' + joint_name +'"')
            # T stays identity
        
        # add the offset in last collumn
        
        T[0][3] = self.offsets[joint_name][0]
        T[1][3] = self.offsets[joint_name][1]
        T[2][3] = self.offsets[joint_name][2]

        print(np.round(T,2))

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                # "chain" all transformations by dot-multiplying them
                T = dot(T,Tl)
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
