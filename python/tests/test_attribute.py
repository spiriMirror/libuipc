# NVIDIA contributions: uipc.view and numpy-view regression coverage for every
# creatable attribute slot class.
#
# The NVIDIA notice applies to the contributions described above, not to the
# pre-existing libuipc or other contributors' code.

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest 
import numpy as np
import uipc
from uipc.geometry import tetmesh
from uipc import view

@pytest.mark.basic 
def test_attribute():
    Vs = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1],
        [0,0,0]])

    Ts = np.array([[0,1,2,3]])

    sc = tetmesh(Vs, Ts)

    name_attr = sc.instances().create("name", "MyString")
    v = view(name_attr)
    v[0] = "hello"
    assert name_attr.view()[0] == "hello", "write then read back instance string attribute"

    sc.instances().resize(10)
    assert len(name_attr.view()) == 10, "resize(10) must grow instance attribute to 10 entries"

@pytest.mark.basic
def test_view_of_every_slot_class():
    # uipc.view(slot) dispatches on the slot class: every numeric class gives
    # a writable array on the slot's memory (the slot is its base), with the
    # dtype of the buffer format (int64 is 'q', uint64 'Q'); a string slot
    # gives its string span.
    sc = tetmesh(np.eye(4, 3), np.array([[0, 1, 2, 3]]))
    values = {
        "Float": (2.5, "d", ()),
        "I32": (np.int32(7), "i", ()),
        "I64": (-(2**40), "q", ()),
        "U64": (2**63 + 5, "Q", ()),
        "Vector2": (np.arange(2.0), "d", (2, 1)),
        "Vector3": (np.arange(3.0), "d", (3, 1)),
        "Vector4": (np.arange(4.0), "d", (4, 1)),
        "Vector6": (np.arange(6.0), "d", (6, 1)),
        "Vector9": (np.arange(9.0), "d", (9, 1)),
        "Vector12": (np.arange(12.0), "d", (12, 1)),
        "Vector2i": (np.arange(2, dtype=np.int32), "i", (2, 1)),
        "Vector3i": (np.arange(3, dtype=np.int32), "i", (3, 1)),
        "Vector4i": (np.arange(4, dtype=np.int32), "i", (4, 1)),
        "Matrix2x2": (np.eye(2), "d", (2, 2)),
        "Matrix3x3": (np.eye(3), "d", (3, 3)),
        "Matrix4x4": (np.eye(4), "d", (4, 4)),
        "Matrix6x6": (np.eye(6), "d", (6, 6)),
        "Matrix9x9": (np.eye(9), "d", (9, 9)),
        "Matrix12x12": (np.eye(12), "d", (12, 12)),
    }
    for name, (value, char, item_shape) in values.items():
        sc.vertices().create(name, value)
        slot = sc.vertices().find(name)
        assert type(slot).__name__ == "AttributeSlot" + name
        v = view(slot)
        assert v.dtype.char == char, name
        assert v.shape == (4,) + item_shape, name
        assert v.base is slot and v.flags.writeable, name
        assert view(slot=slot).ctypes.data == v.ctypes.data, name
        v[...] = 3
        assert np.all(slot.view() == 3), name

    sc.vertices().create("text", "abc")
    span = view(sc.vertices().find("text"))
    assert len(span) == 4 and span[0] == "abc"

    with pytest.raises(TypeError):
        view(sc)
