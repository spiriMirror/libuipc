import pytest

from uipc import builtin


@pytest.mark.basic
@pytest.mark.parametrize(("collection", "uid"), [
    (builtin.ConstitutionUIDCollection, 10),
    (builtin.ImplicitGeometryUIDCollection, 1),
])
def test_registered_uid_info_is_read_only(collection, uid):
    registry = collection.instance()
    info = registry.find(uid)
    assert info.uid == uid
    for field in ("uid", "name", "type", "author", "email", "website", "description", "extras"):
        with pytest.raises(AttributeError):
            setattr(info, field, getattr(info, field))
