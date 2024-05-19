import numpy as np

np.set_printoptions(precision=4)
#Suppress exponential formating
np.set_printoptions(suppress=True)

def trotz(angle, units="rad"):
    """
    Function to get a homogeneous transformation matrix for a rotation around the z-axis.

    Args:
        angle (float): The rotation angle (it can be either in degrees or radians depending on the selected "units").
        units (str, optional): A string indicating if the angle is given in radians "rad" or degrees "deg". Defaults to "rad".

    Returns:
        numpy.ndarray: The homogeneous transformation matrix for a rotation around the z-axis.
    """
    if units == "deg":
        angle = np.deg2rad(angle)
    R = rotz(angle)
    TR = np.append(R, np.array([[0.0], [0.0], [0.0]]), 1)
    TR = np.append(TR, np.array([[0.0, 0.0, 0.0, 1.0]]), 0)
    return TR

def trotx(angle, units="rad"):
    """
    Function to get a homogeneous transformation matrix for a rotation around the x-axis.

    Args:
        angle (float): The rotation angle (it can be either in degrees or radians depending on the selected "units").
        units (str, optional): A string indicating if the angle is given in radians "rad" or degrees "deg". Defaults to "rad".

    Returns:
        numpy.ndarray: The homogeneous transformation matrix for a rotation around the x-axis.
    """
    if units == "deg":
        angle = np.deg2rad(angle)
    R = rotx(angle)
    TR = np.append(R, np.array([[0.0], [0.0], [0.0]]), 1)
    TR = np.append(TR, np.array([[0.0, 0.0, 0.0, 1.0]]), 0)
    return TR


def transl(x, y ,z):
    """
    Function to get a translation matrix.

    Args:
        x (float): The translation along the x-axis.
        y (float): The translation along the y-axis.
        z (float): The translation along the z-axis.

    Returns:
        numpy.ndarray: The translation matrix.
    """
    T = np.array([[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]])
    return T

def rotz(angle, units="rad"):
    """
    Function to get a rotation matrix around the z-axis.

    Args:
        angle (float): The rotation angle (it can be either in degrees or radians depending on the selected "units").
        units (str, optional): A string indicating if the angle is given in radians "rad" or degrees "deg". Defaults to "rad".

    Returns:
        numpy.ndarray: The rotation matrix around the z-axis.
    """
    if units == "deg":
        angle = np.deg2rad(angle)
    #Angle must be in radians
    R = np.array([[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]])
    return R

def rotx(angle, units="rad"):
    """
    Function to get a rotation matrix around the x-axis.

    Args:
        angle (float): The rotation angle (it can be either in degrees or radians depending on the selected "units").
        units (str, optional): A string indicating if the angle is given in radians "rad" or degrees "deg". Defaults to "rad".

    Returns:
        numpy.ndarray: The rotation matrix around the x-axis.
    """
    if units == "deg":
        angle = np.deg2rad(angle)
    #Angle must be in radians
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(angle), -np.sin(angle)], [0.0, np.sin(angle), np.cos(angle)]])
    return Rx

def FK_manipulator(theta, d, a, alpha):
    """
    Function to calculate the forward kinematics of a manipulator.
    """
    HT = trotz(theta, "deg") @ transl(0, 0, d) @ transl(a, 0, 0) @ trotx(alpha, "deg")
    return HT

def rot2rpyfull(R=np.identity(3), units="rad"):
    """
    Function to convert a rotation matrix to roll-pitch-yaw angles considering all possible solutions.

    Args:
        R (numpy.ndarray, optional): The rotation matrix. Defaults to identity matrix.
        units (str, optional): A string indicating if the angles should be returned in radians "rad" or degrees "deg". Defaults to "rad".

    Returns:
        numpy.ndarray: The roll-pitch-yaw angles for all possible solutions.
    """
    r11 = R[0][0]
    r12 = R[0][1]
    r13 = R[0][2]
    r21 = R[1][0]
    r22 = R[1][1]
    r23 = R[1][2]
    r31 = R[2][0]
    
    #yaw angle
    yaw1 = np.arctan2(r21, r11) 
    yaw2 = np.arctan2(-r21, -r11) 
    #roll
    roll1 = np.arctan2(-r23 * np.cos(yaw1) + r13 * np.sin(yaw1), r22 * np.cos(yaw1) - r12 * np.sin(yaw1)) 
    roll2 = np.arctan2(-r23 * np.cos(yaw2) + r13 * np.sin(yaw2), r22 * np.cos(yaw2) - r12 * np.sin(yaw2)) 
    #pitch
    pitch1 = np.arctan2(-r31, r11 * np.cos(yaw1) + r21 * np.sin(yaw1))
    pitch2 = np.arctan2(-r31, r11 * np.cos(yaw2) + r21 * np.sin(yaw2))
    if units == "deg":
        yaw1 = np.rad2deg(yaw1)
        pitch1 = np.rad2deg(pitch1)
        roll1 = np.rad2deg(roll1)
        yaw2 = np.rad2deg(yaw2)
        pitch2 = np.rad2deg(pitch2)
        roll2 = np.rad2deg(roll2)
    
    return np.array([[roll1, pitch1, yaw1], [roll2, pitch2, yaw2]])

def quaternion_from_matrix(data: np.array): 

    rpy = rot2rpyfull(data)
    roll = rpy[0][0]
    pitch = rpy[0][1]
    yaw = rpy[0][2]
    
    roll /= 2.0 
    pitch /= 2.0 
    yaw /= 2.0 

    ci = np.cos(roll) 
    si = np.sin(roll) 
    cj = np.cos(pitch) 
    sj = np.sin(pitch) 
    ck = np.cos(yaw) 
    sk = np.sin(yaw) 

    cc = ci*ck 
    cs = ci*sk 
    sc = si*ck 
    ss = si*sk 

    q = np.empty((4, )) 
    q[0] = cj*sc - sj*cs 
    q[1] = cj*ss + sj*cc 
    q[2] = cj*cs - sj*sc 
    q[3] = cj*cc + sj*ss 
    
    return q 