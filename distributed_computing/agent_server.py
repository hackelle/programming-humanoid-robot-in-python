'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading
from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        print('Starting server...')
        super(ServerAgent, self).__init__()
        # create server under this adress
        # allow_none to call without weard object creation etc 
        self.server = SimpleXMLRPCServer(("localhost", 8000), allow_none=True)
        # register all functions of ServerAgent as RPCs
        self.server.register_instance(self)
        # start new thread for rpc handler
        self.thread = threading.Thread(target = self.server.serve_forever)
        self.thread.start()
        print('done')
        
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        print("get_angle called: " + joint_name)
        
        # YOUR CODE HERE
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        print("set_angle called: " + joint_name + "   angle: " + str(angle))
        # YOUR CODE HERE

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print("get_posture called")

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        print("execute_keyframes called: " + keyframes)
        # YOUR CODE HERE

    def get_transform(self, name):
        '''get transform with given name
        '''
        print("get_transform called: " + name)
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        print("set_transform called: " + effector_name + "    transform:" + str(transform))
        # YOUR CODE HERE

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

