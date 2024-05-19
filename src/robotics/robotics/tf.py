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