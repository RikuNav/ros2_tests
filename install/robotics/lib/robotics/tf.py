import numpy as np

def trotz(angle: float) -> np.array:
    return np.array([[np.cos(angle), -np.sin(angle),  0, 0],
                    [np.sin(angle), np.cos(angle)  ,  0, 0],
                    [0            , 0              ,  1, 0],
                    [0            , 0              ,  0, 1]])

def transz(distance: float) -> np.array:
    return np.array([[1, 0, 0, 0      ],
                    [0, 1, 0, 0       ],
                    [0, 0, 1, distance],
                    [0, 0, 0, 1       ]])

def transx(distance: float) -> np.array:
    return np.array([[1, 0, 0, distance],
                    [0, 1,  0, 0       ],
                    [0, 0,  1, 0       ],
                    [0, 0,  0, 1       ]])

def trotx(angle: float) -> np.array:
    return np.array([[1, 0           , 0             , 0],
                    [0, np.cos(angle), -np.sin(angle), 0],
                    [0, np.sin(angle), np.cos(angle) , 0],
                    [0, 0            , 0             , 1]])  

def rot2rpyfull(rot: np.array) -> np.array:
    y1 = np.arctan2(rot[1, 0], rot[0, 0])
    y2 = np.arctan2(-rot[1, 0], -rot[0, 0])

    r1 = np.arctan2(-rot[1, 2] * np.cos(y1) + rot[0, 2] * np.sin(y1), rot[1, 1] * np.cos(y1) - rot[0, 1] * np.sin(y1)) 
    r2 = np.arctan2(-rot[1, 2] * np.cos(y2) + rot[0, 2] * np.sin(y2), rot[1, 1] * np.cos(y2) - rot[0, 1] * np.sin(y2)) 

    p1 = np.arctan2(-rot[2, 0], rot[0, 0] * np.cos(y1) + rot[1, 0] * np.sin(y1))
    p2 = np.arctan2(-rot[2, 0], rot[0, 0] * np.cos(y2) + rot[1, 0] * np.sin(y2))

    return np.array([[r1, p1, y1], [r2, p2, y2]])