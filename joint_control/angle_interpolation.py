'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import wipe_forehead
import numpy as np

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.init_time = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        if self.init_time == None:
            self.init_time = perception.time
        
        time = perception.time - self.init_time # get time
        names, times, keys = keyframes # unpack keyframes
                       
        target_joints = {} # dict joint_name : value
        
        if time <= 0:
            # get values of first keyframe
            # lambda returns [0][0] of keys
            target_joints = dict(zip(names, map(lambda k: k[0][0], keys)))
            return target_joints
        
        # else
        for i in range(len(names)):
            #interate over all joints
            joint_name = names[i] # joint name
            joint_times = times[i] # array of this joints' times
            joint_keys = keys[i] # array of joints' keyframes
            
            if joint_name not in self.joint_names:
                continue
            
            #find first keyframe after current time
            
            if time < joint_times[0]:
                # before first keyframe-timestamp
                # next keyframe is first keyframe
                next_keyframe = joint_keys[0]
                # last keyframe = pseudo keyframe for nothing happened
                pre_keyframe = [perception.joint[joint_name], [3, 0, 0], [3, 0, 0]]
                pre_time = 0
                next_time = joint_times[0]
                
            elif time > joint_times[len(joint_times)-1]:
                # after last keyframe-timestamp
                # next and last keyframe are the last one
                pre_keyframe = next_keyframe = joint_keys[len(joint_times)-1]
                # times are the times of last frame
                pre_time = next_time = joint_times[len(joint_times)-1]
                
            else:                            
                timestamp_index = 0
                timestamp = joint_times[0]
                
                while timestamp < time and timestamp_index < len(joint_times):
                    timestamp_index += 1
                    timestamp = joint_times[timestamp_index]
                    
                pre_keyframe = joint_keys[timestamp_index-1]
                next_keyframe = joint_keys[timestamp_index]
                pre_time = joint_times[timestamp_index-1]
                next_time = joint_times[timestamp_index]
                
            # the time-angle-pairs
            p0 = (pre_time, pre_keyframe[0])
            p1 = (pre_time + pre_keyframe[2][1], pre_keyframe[0] + pre_keyframe[2][2])
            p2 = (next_time + next_keyframe[1][1], next_keyframe[0] + next_keyframe[1][2])
            p3 = (next_time, next_keyframe[0])
            
            
            # normalize time
            if pre_time == next_time:
                i = 1
            else:
                i = (time-pre_time)/(next_time-pre_time)
             
            t = i
            #t = self.cub_bezier_ang(p0, p1, p2, p3, time)  
                
                
            #add joint after evaluation
            target_joints[joint_name] = self.cub_bezier_eval(p0[1],p1[1],p2[1],p3[1],t)

        return target_joints

    def cub_bezier_eval(self, p0, p1, p2, p3, t):        
        return (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3
    
    
    def cub_bezier_ang(self, p0, p1, p2, p3, time):
        """
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
        solves for t
        use np.roots to solve polynom
        """
        f0 = np.round(p0[0] - time, decimals=4)
        f1 = np.round(-3 * p0[0] + 3 * p1[0], decimals=4)
        f2 = np.round(3 * p0[0] - 6 * p1[0] + 3 * p2[0], decimals=4)
        f3 = np.round(- 1 * p0[0] + 3 * p1[0] -3 * p2[0] + p3[0], decimals=4)
        solve_points = np.roots([f3, f2, f1, f0])

        x = 0
        for solved in solve_points:
            if solved.imag == 0:
                # only exactly one real solution expected
                x = float(solved.real)
                break

        return x


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    #agent.keyframes = leftBackToStand()
    #agent.keyframes = leftBellyToStand()
    #agent.keyframes = rightBackToStand()
    #agent.keyframes = rightBellyToStand()
    #agent.keyframes = wipe_forehead(1)
    agent.run()
