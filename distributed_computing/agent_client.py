'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''


import threading
import xmlrpc.client as rpc
from joint_control.keyframes import leftBackToStand

class PostHandler(object):
    '''The post handler wraps functions to be executed in parallel.
    '''
    def __init__(self, obj):
        self.client = obj

    def execute_keyframes(self, keyframes):
        '''Non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        # Create a new thread to execute the keyframes in parallel
        thread = threading.Thread(target=self.client.execute_keyframes, args=[keyframes])
        thread.start()

    def set_transform(self, effector_name, transform):
        '''Non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        # Create a new thread to set the transform in parallel
        thread = threading.Thread(target=self.client.set_transform, args=[effector_name, transform])
        thread.start()


class ClientAgent(object):
    '''ClientAgent requests RPC service from a remote server
    '''

    # YOUR CODE HERE
    def __init__(self):
        # Initialize the RPC client with the server's address
        self.client = rpc.ServerProxy("http://localhost:6666", allow_none=True)
        self.post = PostHandler(self.client)
        print('Client started')

    def get_angle(self, joint_name):
        '''Get sensor value of the given joint'''
        return self.client.get_angle(joint_name)

    def set_angle(self, joint_name, angle):
        '''Set the target angle of the joint for PID controller
        '''
        return self.client.set_angle(joint_name, angle)

    def get_posture(self):
        '''Return the current posture of the robot'''
        return self.client.get_posture()

    def execute_keyframes(self, keyframes):
        '''Execute keyframes; note that this function is a blocking call,
        e.g., it returns only when keyframes are executed
        '''
        return self.client.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''Get transform with the given name
        '''
        return self.client.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''Solve the inverse kinematics and control joints using the results 'blocking'
        '''
        return self.client.set_transform(effector_name, transform)


if __name__ == '__main__':
    # Instantiate a ClientAgent
    agent = ClientAgent()
    # TEST CODE HERE
    # Execute the predefined keyframes to test the functionality
    agent.execute_keyframes(leftBackToStand())
