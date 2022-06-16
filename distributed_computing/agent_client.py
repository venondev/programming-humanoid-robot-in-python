'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

from concurrent.futures import thread
import threading
import requests
import weakref
from keyframes import hello, wipe_forehead, leftBackToStand

from sympy import arg

class PostHandler(object):
    ''' the post hander wraps function to be executed in parallel
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.execute_keyframes, args=(keyframes,))
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.set_transform, args=([effector_name, transform],))
        thread.start()

class RPCRequest():

    def __init__(self, host) -> None:
        self.host = host

    def __getattr__(self, param):
        def start_req(*args, **kwargs):
            return requests.post(self.host, json={
                "method": param,
                "params": args,
                "jsonrpc": "2.0",
                "id": 0,
            }).json()["result"]

        return start_req


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.rpc = RPCRequest("http://localhost:4000")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.rpc.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.rpc.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.rpc.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        print("Called")
        self.rpc.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.rpc.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        return self.rpc.set_transform(effector_name, transform)

if __name__ == '__main__':
    # agent = ClientAgent()
    # # TEST CODE HERE

    a = ClientAgent()

    print("Start")
    print(a.post.execute_keyframes(hello()))
    print("Finished")


