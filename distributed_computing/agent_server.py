"""In this file you need to implement remote procedure call (RPC) server
* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
"""


# Import necessary libraries and modules
import os
import sys
import threading
import pickle
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler

# Importing the InverseKinematicsAgent from the kinematics module
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
from kinematics.inverse_kinematics import InverseKinematicsAgent

# Define a custom request handler for the XML-RPC server
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

# Define the ServerAgent class, which provides the RPC service
class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''

    def __init__(self):
        super(ServerAgent, self).__init__()

        # Load the posture classifier from a pickle file
        robot_pose_data = open(r'..\joint_control\robot_pose.pkl', 'rb')
        self.posture_classifier = pickle.load(robot_pose_data)
        self.posture = 'unknown'
        robot_pose_data.close()

        # Create an XML-RPC server on localhost and port 6666
        self.server = SimpleXMLRPCServer(('localhost', 6666), requestHandler=RequestHandler, allow_none=True)
        self.server.register_introspection_functions()
        self.server.register_multicall_functions()
        self.server.register_instance(self)

        # Start a thread for serving RPC requests
        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.start()

        # Display a message indicating that the server is starting
        print('starting server ...')

    def think(self, perception):
        # Recognize the current posture based on perception data
        self.posture = self.recognize_posture(perception)
        return super(ServerAgent, self).think(perception)

    def recognize_posture(self, perception):
        # Recognize the posture using a trained classifier
        posture = 'unknown'
        postures = ['HeadBack', 'Left', 'Right', 'Crouch', 'Knee', 'Stand', 'Sit', 'StandInit', 'Frog', 'Back', 'Belly']
        joint_names = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch',
                       'RKneePitch']
        values = []

        # Gather joint values and IMU data for posture recognition
        for joint_name in joint_names:
            values.append(perception.joint[joint_name])
        values += perception.imu
        ID = self.posture_classifier.predict([values])[0]
        posture = postures[ID]
        return posture

    def get_angle(self, joint_name):
        """get sensor value of given joint"""
        return self.perception.joint.get(joint_name)

    def set_angle(self, joint_name, angle):
        """set target angle of joint for PID controller
        """
        self.target_joints[joint_name] = angle

    def get_posture(self):
        """return current posture of robot"""
        return str(self.posture)

    def execute_keyframes(self, keyframes):
        """execute keyframes; note that this function is a blocking call,
        i.e., it returns only after keyframes are executed
        """
        # YOUR CODE HERE
        self.keyframes = keyframes

    def get_transform(self, name):
        """get transform with the given name
        """
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        """solve the inverse kinematics and control joints using the results
        """
        self.transforms[effector_name] = transform

# Entry point for the script
if __name__ == '__main__':
    # Instantiate and run the ServerAgent
    agent = ServerAgent()
    agent.run()
