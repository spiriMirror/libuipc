from types import SimpleNamespace

import pytest


warp = pytest.importorskip("warp")
WarpBuffer = pytest.importorskip("uipc.adapter.warp").WarpBuffer


@pytest.mark.basic
def test_empty_strides_fall_back_to_element_size():
    array = SimpleNamespace(strides=(), dtype=warp.float32)

    assert WarpBuffer.element_stride(array) == WarpBuffer.element_size(array)
