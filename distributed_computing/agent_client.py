'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpclib

import threading
from numpy import zeros

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        temp_thread = Thread(target=self.proxy.execute_keyframes, args=[keyframes])
        # make thread a daemon in case of ctrl c stops
        temp_thread.daemon = True
        temp_thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        temp_thread = Thread(target=self.proxy.set_transform, args=[effector_name, transform])
        # make thread a daemon in case of ctrl c stops 
        temp_thread.daemon = True
        temp_thread.start()

class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    def __init__(self):        
        self.post = PostHandler(self)
        # create object for packing and unpacking messages
        self.pack_unpack = PackUnpack()
        # set proxy 
        self.proxy = xmlrpclib.ServerProxy("http://localhost:8000/")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        print("Getting angle for " + joint_name)
        return self.proxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        print("Setting angle for " + joint_name + " to " + str(angle))
        self.proxy.set_angle(joint_name, angle)
        return

    def get_posture(self):
        '''return current posture of robot'''
        print("Getting posture")
        return self.proxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        print("Executing keyframes ...")
        self.proxy.execute_keyframes(keyframes)
        print("done")
        return

    def get_transform(self, name):
        '''get transform with given name
        '''
        print("Getting transform for " + name)
        return self.pack_unpack.unpack(self.proxy.get_transform(name))

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        print("Setting transform for " + effector_name + " to " + str(transform) + " ...")
        self.proxy.set_transform(effector_name, self.pack_unpack.pack(transform))
        print("done")
        return

#make it a class for easy import to server
class PackUnpack(object):

    def pack(self, transformation):
        print("packing ...")
        values = []
        for i in range(4):
            for j in range(4):
                values.append(transformation[i, j].item())
        print("done")
        return values

    def unpack(self, values):
        print("unpacking...")
        transformation = zeros([4, 4])
        ind = 0
        for i in range(4):
            for j in range(4):
                transformation[i, j] = values[ind]
                ind += 1
        print("done")
        return transformation

if __name__ == '__main__':
    agent = ClientAgent()
    
    print(agent.get_transform("RAnkleRoll"))


