import numpy as np

def quaternion_from_matrix(matrix: np.array):
    m00=matrix[0,0]
    m01=matrix[0,1]
    m02=matrix[0,2]
    m10=matrix[1,0]
    m11=matrix[1,1]
    m12=matrix[1,2]
    m20=matrix[2,0]
    m21=matrix[2,1]
    m22=matrix[2,2]

    qw = np.sqrt(1 + m00 + m11 + m22) / 2
    qx = (m21 - m12) / (4 * qw)
    qy = (m02 - m20) / (4 * qw)
    qz = (m10 - m01) / (4 * qw)

    return np.array([qw, qx, qy, qz])