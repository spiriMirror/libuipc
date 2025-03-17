import numpy as np
import uipc
import pytest 
import pytetwild
import numpy as np
import uipc
from uipc import constitution

@pytest.mark.basic 
def test_tetwild():
    vertices = np.array([[-1., -1., -1.],
                        [-1., -1.,  1.],
                        [-1.,  1., -1.],
                        [-1.,  1.,  1.],
                        [ 1., -1., -1.],
                        [ 1., -1.,  1.],
                        [ 1.,  1., -1.],
                        [ 1.,  1.,  1.]])

    faces = np.array([[0., 1., 3., 2.],
                    [2., 3., 7., 6.],
                    [6., 7., 5., 4.],
                    [4., 5., 1., 0.],
                    [2., 6., 4., 0.],
                    [7., 3., 1., 5.]])

    v_out, tetra = pytetwild.tetrahedralize(vertices, faces, optimize=False, edge_length_fac=0.5)

    for tet in tetra:
        e0 = v_out[tet[1]] - v_out[tet[0]]
        e1 = v_out[tet[2]] - v_out[tet[0]]
        e2 = v_out[tet[3]] - v_out[tet[0]]
        
        D = np.dot(e0, np.cross(e1, e2))
        if(D < 0):
            print(f'Inverting tetrahedron {tet}, D={D}')
            tet[0], tet[1] = tet[1], tet[0]
            print(f'corrected tetrahedron {tet}')

    tet = uipc.geometry.tetmesh(v_out, tetra)
    uipc.geometry.label_surface(tet)
    surf = uipc.geometry.extract_surface(tet)

    abd = constitution.AffineBodyConstitution()
    abd.apply_to(tet, 1e8)