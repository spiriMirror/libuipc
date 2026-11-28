import numpy as np
import pytest
import sys

from uipc.cli import mesh_doctor as cli
from uipc.constitution import DiscreteShellBending, StableNeoHookean
from uipc.dev.mesh_doctor import MeshDoctor
from uipc.geometry import pointcloud, tetmesh


def test_fem_doctor_checks_rest_geometry_without_applying_material_to_source(tmp_path):
    doctor = MeshDoctor(str(tmp_path))
    model = StableNeoHookean()
    points = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    mesh = tetmesh(points, np.array([[0, 1, 2, 3]], np.int32))
    assert doctor.check_mesh(model, mesh)
    assert mesh.meta().find("constitution_uid") is None
    assert not doctor.check_mesh(model, pointcloud(points))
    assert not doctor.check_mesh(model, tetmesh(points, np.array([[0, 2, 1, 3]], np.int32)))
    with pytest.raises(NotImplementedError, match="base FEM material"):
        doctor.check_mesh(DiscreteShellBending(), mesh)


@pytest.mark.parametrize("options", [[], ["--no-gui"]])
def test_mesh_doctor_cli_runs_headless(tmp_path, monkeypatch, options):
    mesh_file = tmp_path / "tet.obj"
    mesh_file.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
                         "f 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n")
    monkeypatch.setitem(sys.modules, "polyscope", None)
    monkeypatch.setattr(sys, "argv", ["mesh-doctor", str(mesh_file), "--abd", *options])
    with pytest.raises(SystemExit) as exit_status:
        cli.main()
    assert exit_status.value.code == 0
