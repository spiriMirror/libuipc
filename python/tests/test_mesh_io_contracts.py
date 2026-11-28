import numpy as np
import pytest

from uipc import Scene
from uipc.geometry import SimplicialComplexIO, UrdfIO, linemesh, pointcloud, trimesh


def test_obj_polyline_and_msh_element_types(tmp_path):
    io = SimplicialComplexIO()
    chain = tmp_path / "chain.obj"
    chain.write_text("v 0 0 0\nv 1 0 0\nv 2 0 0\nv 3 0 0\nl 1 2 3 4\n")
    mesh = io.read(str(chain))
    np.testing.assert_array_equal(mesh.edges().topo().view()[..., 0], [[0, 1], [1, 2], [2, 3]])
    v = np.eye(3)
    for sc, element in [
        (pointcloud(v[:1]), "1 15 0 1"),
        (linemesh(v[:2], np.array([[0, 1]])), "1 1 0 1 2"),
        (trimesh(v, np.array([[0, 1, 2]])), "1 2 0 1 2 3"),
    ]:
        output = tmp_path / "mesh.msh"
        io.write(filename=str(output), sc=sc)
        records = output.read_text().split("$Elements\n")[1].split("$EndElements")[0]
        assert records.splitlines() == ["1", element]


@pytest.mark.parametrize("visual", ["", "<geometry><box size='1 1 1'/></geometry>",
                                    "<geometry><mesh filename='tri.obj'/></geometry>"])
def test_urdf_geometry_origins_and_optional_visual(tmp_path, visual):
    (tmp_path / "tri.obj").write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    collision = "<collision><origin xyz='1 0 0'/><geometry><mesh filename='tri.obj'/></geometry></collision>"
    visual_tag = f"<visual><origin xyz='2 0 0'/>{visual}</visual>" if visual else ""
    path = tmp_path / "hand.urdf"
    path.write_text(
        f"<robot name='robot'><link name='base'>{collision}{visual_tag}</link><link name='child'/>"
        "<joint name='j' type='revolute'><parent link='base'/><child link='child'/>"
        "<origin xyz='0 3 0'/><axis xyz='0 0 1'/><limit lower='-1' upper='1' effort='1' velocity='1'/>"
        "</joint></robot>"
    )
    scene = Scene()
    robot = scene.objects().create("robot")
    controller = UrdfIO().read(robot, str(path))
    slot, = controller.links()
    geom = slot.geometry()
    np.testing.assert_allclose(geom.transforms().view()[0] @ [1, 0, 0, 1], [0, 0, -1, 1])
    np.testing.assert_array_equal(geom.positions().view()[0, :, 0], [1, 0, 0])
    has_visual_mesh = "mesh filename" in visual
    assert geom.meta().find("is_visible").view()[0] == int(not has_visual_mesh)
    controller.sync_visual_mesh()
    if has_visual_mesh:
        vis, _ = scene.geometries().find(slot.id() + 1)
        np.testing.assert_array_equal(vis.geometry().positions().view()[0, :, 0], [2, 0, 0])
        np.testing.assert_array_equal(vis.geometry().transforms().view(), geom.transforms().view())
    joints = controller.revolute_joints().geometry().instances()
    np.testing.assert_array_equal(joints.find("link_ids").view().ravel(), [slot.id(), -1])
    np.testing.assert_allclose(joints.find("transform").view()[0, :3, 3], [-3, 0, 0])
    controller.move_root(np.array([0., 0., 2.]), np.zeros(3))
    controller.apply_to("transform")
    np.testing.assert_allclose(joints.find("transform").view()[0, :3, 3], [-3, 2, 0])
