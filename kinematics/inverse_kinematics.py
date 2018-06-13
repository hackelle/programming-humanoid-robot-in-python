'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, dot, array
import numpy
from math import atan2
from scipy.optimize import fmin


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: dict of joint angles
        '''       
        start = [0] * len(self.chains[effector_name])

        print 'Opt...',
        optimization = fmin(self.err_func, start, args=(effector_name, transform))
        print 'done!'

        joint_angles = dict(zip(self.chains[effector_name], optimization))
        return joint_angles
    
    def err_func(self, thetas, effector_name, target):
        '''calculate the error of effector_name
        used in fmin, should be 0 for perfect match
        '''
        
        # devide into rotations and translations of current state
        Ts = self.forward_kinematics_for_inverse(effector_name, thetas)
        rot_trans_is = self.from_trans(Ts)
        # do same for target
        rot_trans_target = self.from_trans(target)
        
        # calculate and weight error
        error = rot_trans_target - rot_trans_is
        weight_error = numpy.sum(error[:2]) + 10*numpy.sum(error[3:])
        
        return weight_error
    
    def forward_kinematics_for_inverse(self, effector_name, thetas):
        # calculate forward_kinematics for just one chain
        T = identity(4)
        for i, joint in enumerate(self.chains[effector_name]):
            angle = thetas[i]
            Tl = self.local_trans(joint, angle)
            T = dot(T, Tl)
        return T
        

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        # get angles
        # using a dict to make it easier
        angles = self.inverse_kinematics(effector_name, transform)
        
        name = list()
        times = list()
        keys = list()

        for chain in self.chains:
            # only set for chain of effector_name
            # for rest, set angles to 0
            
            # use two times for a smooth movement
                        
            if chain == effector_name:
                for joint in self.chains[chain]:
                    name.append(joint)
                    keys.append([[        0, [3, -1, 0.], [3, 1, 0.]],
                                 [angles[joint], [3, -1, 0.], [3, 1, 0.]]])
                    times.append([5.0, 8.0])
            else:
                for joint in self.chains[chain]:
                    name.append(joint)
                    keys.append([[0, [3, -1, 0.], [3, 1, 0.]],
                                 [0, [3, -1, 0.], [3, 1, 0.]]])
                    times.append([5.0, 8.0])


        self.keyframes = (name, times, keys)  # the result joint angles have to fill in
        
    def from_trans(self, transform):
        '''get x,y,z coordinates and rotations around x,y,z from a transformation
        '''
        x, y, z = T[0, -1], T[1, -1], T[2, -1]
        theta_x, theta_y, theta_z = 0, 0, 0

        if T[0, 0] == 1:
            theta_x = atan2(T[2, 1], T[1, 1])
        elif T[1, 1] == 1:
            theta_y = atan2(T[0, 2], T[0, 0])
        elif T[2, 2] == 1:
            theta_z = atan2(T[1, 0], T[0, 0])

        return numpy.array([x, y, z, theta_x, theta_y, theta_z])
    
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
