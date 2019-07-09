import numpy as np

def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def intersects(A, B, C, D):
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

def rotate(v, radians):
    c, s = np.cos(radians), np.sin(radians)
    return np.array([v[0] * c - v[1] * s, v[0] * s + v[1] * c])

