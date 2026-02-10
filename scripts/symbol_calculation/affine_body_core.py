import sys
sys.path.append('../')
from SymEigen import *
import sympy as sp

def compute_J_point(xbar):
    J = sp.Matrix.zeros(3, 12)
    J[0:3, 0:3] = sp.eye(3)
    J[0, 3:6] = xbar.T
    J[1, 6:9] = xbar.T
    J[2, 9:12] = xbar.T
    return J


def compute_J_vec(xbar):
    J = sp.Matrix.zeros(3, 12)
    # Remove the translation part
    # we only need the rotation / scaling part for vector transformation
    # J[0:3, 0:3] = sp.eye(3)
    J[0, 3:6] = xbar.T
    J[1, 6:9] = xbar.T
    J[2, 9:12] = xbar.T
    return J