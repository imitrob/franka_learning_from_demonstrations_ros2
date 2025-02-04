import numpy as np
import quaternion
from quaternion_algebra.algebra import inner, change_sign, quaternion_power, quaternion_product, quaternion_divide, quaternion_log, quaternion_absolute
def slerp_with_power(q1, q2, tau):
    if inner(q1, q2) < 0:
        q1 = change_sign(q1)
    q_slerp = quaternion_product(quaternion_power(quaternion_divide(q2, q1), tau), q1)
    return q_slerp

def slerp(q1, q2, tau):
    if inner(q1, q2) < 0:
        q1 = change_sign(q1)
    theta = np.arccos(np.abs(inner(q1, q2)))
    q_slerp = np.zeros(4)

    q_slerp.x = (np.sin((1 - tau) * theta) * q1.x + np.sin(tau * theta) * q2.x) / np.sin(theta)
    q_slerp.y = (np.sin((1 - tau) * theta) * q1.y + np.sin(tau * theta) * q2.y) / np.sin(theta)
    q_slerp.z = (np.sin((1 - tau) * theta) * q1.z + np.sin(tau * theta) * q2.z) / np.sin(theta)
    q_slerp.w = (np.sin((1 - tau) * theta) * q1.w + np.sin(tau * theta) * q2.w) / np.sin(theta)
    return q_slerp

def slerp_sat(q1, q2, theta_max):
    if inner(q1, q2) < 0:
        q1 = change_sign(q1)
    theta = np.arccos(np.abs(inner(q1, q2)))
    q_slerp = np.copy(q1)
    if theta > theta_max:
        q_slerp.w = (np.sin(theta - theta_max) * q1.w + np.sin(theta_max) * q2.w) / np.sin(theta)
        q_slerp.x = (np.sin(theta - theta_max) * q1.x + np.sin(theta_max) * q2.x) / np.sin(theta)
        q_slerp.y = (np.sin(theta - theta_max) * q1.y + np.sin(theta_max) * q2.y) / np.sin(theta)
        q_slerp.z = (np.sin(theta - theta_max) * q1.z + np.sin(theta_max) * q2.z) / np.sin(theta)
    return q_slerp


def intrinsic(q1, q2):
    """Geodesic distance between rotations within the SO(3) manifold.

        This function is equivalent to

            min(
                np.absolute(np.log(q1 / q2)),
                np.absolute(np.log(q1 / -q2))
            )

        which is a measure of the "distance" between two rotations.  Note
        that no normalization is performed, which means that if q1 and/or
        q2 do not have unit norm, this is a more general type of distance.
        If they are normalized, the result of this function is half the
        angle through which vectors rotated by q1 would need to be rotated
        to lie on the same vectors rotated by q2.

        Parameters
        ----------
        q1, q2 : QuaternionicArray
            Quaternionic arrays to be measured

        See also
        --------
        quaternionic.distance.rotor.chordal
        quaternionic.distance.rotor.intrinsic
        quaternionic.distance.rotation.chordal

        """
    qtemp = np.quaternion()
    a = (q1.w - q2.w)**2 + (q1.x - q2.x)**2 + (q1.y - q2.y)**2 + (q1.z - q2.z)**2
    b = (q1.w + q2.w)**2 + (q1.x + q2.x)**2 + (q1.y + q2.y)**2 + (q1.z + q2.z)**2
    if b > a:
        qtemp = quaternion_divide(q1, q2)
    else:
        qtemp = quaternion_divide(q1, -q2)
    qtemp = quaternion_log(qtemp)
    qout = quaternion_absolute(qtemp)
    qout *= 2
    return qout




def normalize(q):
    """Normalize a quaternion to unit length."""
    return q / np.linalg.norm(q)

def slerp2(q1, q2, p, epsilon=1e-8):
    """
    Perform Spherical Linear Interpolation (SLERP) between two quaternions.
    
    Args:
        q1: First quaternion (4-element array-like).
        q2: Second quaternion (4-element array-like).
        p: Interpolation weight (0 <= p <= 1).
        epsilon: Threshold to prevent division by small numbers.
    
    Returns:
        Interpolated quaternion (numpy array).
    """
    q1 = normalize(np.array(q1, dtype=np.float64))
    q2 = normalize(np.array(q2, dtype=np.float64))
    
    # Compute the dot product to determine the angle between quaternions
    dot = np.dot(q1, q2)
    
    # Ensure shortest path (correct for negative dot product)
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    
    # If the quaternions are almost the same, use linear interpolation
    if dot > 1.0 - epsilon:
        result = q1 + p * (q2 - q1)
        return normalize(result)
    
    # Compute the angle between the quaternions
    theta_0 = np.arccos(dot)
    theta = theta_0 * p
    
    # Compute the interpolated quaternion components
    q3 = normalize(q2 - q1 * dot)  # Orthogonal component
    
    return q1 * np.cos(theta) + q3 * np.sin(theta)

