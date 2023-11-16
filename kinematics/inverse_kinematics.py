from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        # Initialize joint angles randomly
        joint_angles = np.random.random(len(self.chains[effector_name]))

        # Hyperparameters for inverse kinematics
        lambda_ = 1
        max_step = 0.1

        # Target position for the end effector
        target = np.matrix([self.from_trans(transform)]).T

        # Iteratively solve inverse kinematics
        while True:
            # Initialize transformations for the joints
            Ts = [identity(len(self.chains[effector_name]))]
            for name in self.chains[effector_name]:
                Ts.append(self.transforms[name])

            # Calculate end effector transformation
            Te = np.matrix([self.from_trans(Ts[-1])]).T

            # Calculate the error between the target and actual end effector position
            error = target - Te
            error[error > max_step] = max_step
            error[error < -max_step] = -max_step

            # Calculate the Jacobian matrix
            T = np.matrix([self.from_trans(j) for j in Ts[0:-1]]).T
            J = Te - T
            dT = Te - T

            J[0, :] = dT[2, :]
            J[1, :] = dT[1, :]
            J[2, :] = dT[0, :]
            J[-1, :] = 1

            # Update joint angles using the pseudo-inverse of the Jacobian
            d_theta = np.dot(lambda_, np.dot(np.linalg.pinv(J), error))
            joint_angles += np.asarray(d_theta.T)[0]

            # Break the loop if the change in joint angles is small enough
            if np.linalg.norm(d_theta) < 1e-4:
                break

        return joint_angles

    def set_transforms(self, effector_name, transform):
        # Set the transforms using the results of inverse kinematics
        angles = self.inverse_kinematics(effector_name, transform)
        names = []
        times = []
        keys = []

        # Create keyframes for joint control
        for i, joint in enumerate(self.chains[effector_name]):
            names.append(joint)
            times.append([5.0, 7.0])
            keys.append([[angles[i], [2, 0, 0], [2, 0, 0]],
                         [angles[i], [1, 0, 0], [1, 0, 0]]])

        self.keyframes = (names, times, keys)


if __name__ == '__main__':
    # Create an instance of InverseKinematicsAgent
    agent = InverseKinematicsAgent()

    # Test inverse kinematics with a specific end effector transformation
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
