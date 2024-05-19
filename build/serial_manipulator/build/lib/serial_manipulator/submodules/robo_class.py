import numpy as np
from .functions import *
import copy

# Class definition
class SimpleRobot:
    def __init__(self, **kwargs):
        self.offsets = np.empty((1, 2), dtype=object)

        for key, value in kwargs.items():
            self.add_offsets(value[0], value[1])

        self.offsets = np.delete(self.offsets, 0, 0)
        self.dh_matrix = [[0] for i in range(len(self.offsets))]

        for i in range(len(self.offsets)):
            self.dh_matrix[i] = copy.deepcopy(self.offsets[i, 1])

        self.compute_fk()
        self.print_output(first=True)

    def __call__(self, q_values):       
        if len(q_values) != len(self.dh_matrix):
            raise ValueError("The number of joints and dh_matrix must be the same")
        
        joints = np.arange(1, len(self.dh_matrix) + 1)

        for i in range(len(joints)):
            self.move(joints[i], q_values[i])
        
        self.compute_fk()
        self.print_output()

    def move(self, joint, q_value):
        joint -= 1
        copy_offsets = copy.deepcopy(self.offsets)
        if self.offsets[joint, 0] == "R": 
            self.dh_matrix[joint] = copy_offsets[joint, 1]
            self.dh_matrix[joint][0] = copy_offsets[joint, 1][0] + q_value
    
        elif self.offsets[joint, 0] == "P":
            self.dh_matrix[joint] = copy_offsets[joint, 1]
            self.dh_matrix[joint][1] = copy_offsets[joint, 1][1] + q_value

    def compute_fk(self):
        self.outputs = np.empty((len(self.dh_matrix), 4, 4))
        self.output = np.eye(4)

        for i in range(len(self.dh_matrix)):
            self.outputs[i] = FK_manipulator(self.dh_matrix[i][0], self.dh_matrix[i][1], self.dh_matrix[i][2], self.dh_matrix[i][3])
            self.output = self.output @ self.outputs[i]

        self.rpy = np.rad2deg(rot2rpyfull(self.output))

    def add_offsets(self, key, value):
        data = np.array([key, value], dtype=object)
        self.offsets = np.vstack((self.offsets, data))

    def print_output(self, first=False):
        if first:
            print('-' * 50)
            print('The initial DH parameters are:')
            print(self.offsets)
            print('---')
            print('The Base Configuration is:')
            print(self.output)
            print('---')
            print('The RPY angles are (°):')
            print(self.rpy)
            print('-' * 50)    
        
        else:
            print('-' * 50)
            print('The Forward Kinematics are:')
            print(self.output)
            print('---')
            print('The RPY angles are (°):')
            print(self.rpy)
            print('-' * 50)

# # Example of use
# robot_offsets = {'1': ['R', [0, 243.3, 0, -90]],
#                  '2': ['R', [-90, 0, 200, 180]],
#                  '3': ['R', [-90, 0, 87, 90]],
#                  '4': ['R', [0, 227.6, 0, 90]],
#                  '5': ['R', [0, 0, 0, -90]],
#                  '6': ['R', [0, 61.5, 0, 0]]}

# # Create the robot
# robotsito = SimpleRobot(**robot_offsets)

# # Move the robot
# robotsito([0, 9.7, 33.4, 0, -21.5, 0])