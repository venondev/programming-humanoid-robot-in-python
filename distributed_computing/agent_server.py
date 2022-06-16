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
from ntpath import join
import os
import sys
import threading
from time import sleep
import numpy as np

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))


from inverse_kinematics import InverseKinematicsAgent

from werkzeug.wrappers import Request, Response
from werkzeug.serving import run_simple

from jsonrpc import JSONRPCResponseManager, dispatcher


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE

    def __init__(self) -> None:
        super().__init__()

        rpc = threading.Thread(target=self.init_server)
        rpc.start()

    def init_server(self):
        run_simple('localhost', 4000, self.application)

    @Request.application
    def application(self, request):
        # Dispatcher is dictionary {<method_name>: callable}
        dispatcher.add_method(self.get_angle, "get_angle")
        dispatcher.add_method(self.set_angle, "set_angle")
        dispatcher.add_method(self.get_posture, "get_posture")
        dispatcher.add_method(self.execute_keyframes, "execute_keyframes")
        dispatcher.add_method(self.get_transform, "get_transform")
        dispatcher.add_method(self.set_transform, "set_transform")

        response = JSONRPCResponseManager.handle(
            request.data, dispatcher)
        return Response(response.json, mimetype='application/json')
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        self.is_init = True
        self.motion_finished = False
        while (not self.motion_finished):
            sleep(0.5)
        return

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, np.array(transform))

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

