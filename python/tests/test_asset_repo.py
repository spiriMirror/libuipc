"""Tests for uipc.asset.AssetRepo.

The tests exercise the pure-Python logic (construction, repr, pattern
filtering, error handling) **without** requiring the ``huggingface_hub``
package to be installed or any network access.  Hub interactions are
replaced with lightweight stubs injected via ``monkeypatch``.
"""

import sys
import types
import pathlib
import importlib
import importlib.util

import pytest

# ---------------------------------------------------------------------------
# Bootstrap: load uipc.asset without the native uipc extension
# ---------------------------------------------------------------------------

_ASSET_PATH = pathlib.Path(__file__).parent.parent / "src" / "uipc" / "asset.py"


def _load_asset_module():
    """Load uipc/asset.py with a minimal stub for the uipc package."""
    if "uipc" not in sys.modules:
        stub = types.ModuleType("uipc")
        stub.__path__ = []
        sys.modules["uipc"] = stub

    spec = importlib.util.spec_from_file_location("uipc.asset", str(_ASSET_PATH))
    mod = importlib.util.module_from_spec(spec)
    sys.modules["uipc.asset"] = mod
    spec.loader.exec_module(mod)
    return mod


_asset_mod = _load_asset_module()
AssetRepo = _asset_mod.AssetRepo
DEFAULT_REPO_ID = _asset_mod.DEFAULT_REPO_ID
DEFAULT_REPO_TYPE = _asset_mod.DEFAULT_REPO_TYPE

# ---------------------------------------------------------------------------
# Stub helpers for huggingface_hub
# ---------------------------------------------------------------------------


class _StubSibling:
    """Mimics ``huggingface_hub.hf_api.RepoSibling``."""

    def __init__(self, rfilename, size=1024, lfs=None):
        self.rfilename = rfilename
        self.size = size
        self.lfs = lfs


class _StubRepoInfo:
    """Mimics the object returned by ``HfApi.repo_info()``."""

    def __init__(self, siblings):
        self.siblings = siblings


class _StubHfApi:
    """Minimal stand-in for ``huggingface_hub.HfApi``."""

    _files = [
        "README.md",
        "meshes/cube.obj",
        "meshes/sphere.obj",
        "papers/ipc2020.pdf",
        "configs/default.json",
    ]
    _siblings = [_StubSibling(f, size=100 * (i + 1)) for i, f in enumerate(_files)]

    def __init__(self, token=None):
        self.token = token

    def list_repo_files(self, repo_id, repo_type=None, revision=None):
        return list(self._files)

    def repo_info(self, repo_id, repo_type=None, revision=None):
        return _StubRepoInfo(self._siblings)


def _make_hub_stub():
    """Return a fake ``huggingface_hub`` module."""
    hub = types.ModuleType("huggingface_hub")
    hub.HfApi = _StubHfApi

    def _hf_hub_download(
        repo_id, filename, repo_type=None, cache_dir=None, local_dir=None, token=None, revision=None
    ):
        # Return a deterministic path based on local_dir or /tmp
        base = pathlib.Path(local_dir) if local_dir else pathlib.Path("/tmp/hf_cache")
        return str(base / filename)

    hub.hf_hub_download = _hf_hub_download
    return hub


@pytest.fixture(autouse=True)
def _inject_hub_stub(monkeypatch):
    """Inject the stub ``huggingface_hub`` into ``sys.modules`` for every test."""
    stub = _make_hub_stub()
    monkeypatch.setitem(sys.modules, "huggingface_hub", stub)


# ---------------------------------------------------------------------------
# Construction & repr tests
# ---------------------------------------------------------------------------


@pytest.mark.basic
def test_default_construction():
    repo = AssetRepo()
    assert repo.repo_id == DEFAULT_REPO_ID
    assert repo.repo_type == DEFAULT_REPO_TYPE
    assert repo.revision is None


@pytest.mark.basic
def test_custom_construction():
    repo = AssetRepo(
        repo_id="myorg/myrepo",
        repo_type="model",
        revision="v1.0",
        token="hf_token",
        cache_dir="/tmp/cache",
    )
    assert repo.repo_id == "myorg/myrepo"
    assert repo.repo_type == "model"
    assert repo.revision == "v1.0"


@pytest.mark.basic
def test_repr_default():
    repo = AssetRepo()
    r = repr(repo)
    assert "AssetRepo(" in r
    assert DEFAULT_REPO_ID in r


@pytest.mark.basic
def test_repr_custom():
    repo = AssetRepo(repo_id="a/b", repo_type="model", revision="main")
    r = repr(repo)
    assert "a/b" in r
    assert "model" in r
    assert "main" in r


# ---------------------------------------------------------------------------
# list_files tests
# ---------------------------------------------------------------------------


@pytest.mark.basic
def test_list_files_all():
    repo = AssetRepo()
    files = repo.list_files()
    assert len(files) == 5
    assert files == sorted(files)


@pytest.mark.basic
def test_list_files_pattern():
    repo = AssetRepo()
    objs = repo.list_files(pattern="*.obj")
    assert all(f.endswith(".obj") for f in objs)
    assert len(objs) == 2


@pytest.mark.basic
def test_list_files_pattern_no_match():
    repo = AssetRepo()
    result = repo.list_files(pattern="*.xyz")
    assert result == []


@pytest.mark.basic
def test_list_files_subdir_pattern():
    repo = AssetRepo()
    pdfs = repo.list_files(pattern="papers/*")
    assert pdfs == ["papers/ipc2020.pdf"]


# ---------------------------------------------------------------------------
# download tests
# ---------------------------------------------------------------------------


@pytest.mark.basic
def test_download_returns_path():
    repo = AssetRepo()
    p = repo.download("meshes/cube.obj")
    assert "meshes/cube.obj" in p


@pytest.mark.basic
def test_download_with_local_dir(tmp_path):
    repo = AssetRepo()
    p = repo.download("meshes/cube.obj", local_dir=str(tmp_path))
    assert str(tmp_path) in p


@pytest.mark.basic
def test_download_files_pattern():
    repo = AssetRepo()
    paths = repo.download_files("meshes/*.obj")
    assert len(paths) == 2
    assert paths == sorted(paths)


# ---------------------------------------------------------------------------
# file_info tests
# ---------------------------------------------------------------------------


@pytest.mark.basic
def test_file_info_found():
    repo = AssetRepo()
    info = repo.file_info("README.md")
    assert info["filename"] == "README.md"
    assert isinstance(info["size"], int)
    assert "lfs" in info


@pytest.mark.basic
def test_file_info_not_found():
    repo = AssetRepo()
    with pytest.raises(FileNotFoundError, match="no_such_file"):
        repo.file_info("no_such_file.txt")


# ---------------------------------------------------------------------------
# Import-error handling
# ---------------------------------------------------------------------------


@pytest.mark.basic
def test_import_error_when_hub_missing(monkeypatch):
    """Operations raise a clear ImportError when huggingface_hub is absent."""
    monkeypatch.delitem(sys.modules, "huggingface_hub", raising=False)

    # Patch the helper so it tries a fresh import (which will fail because
    # the stub has been removed and the real package is not installed).
    def _failing_require():
        try:
            import huggingface_hub
            return huggingface_hub
        except ImportError:
            raise ImportError(
                "The 'huggingface_hub' package is required for asset management. "
                "Install it with:  pip install huggingface_hub"
            ) from None

    monkeypatch.setattr(_asset_mod, "_require_huggingface_hub", _failing_require)

    repo = AssetRepo()
    with pytest.raises(ImportError, match="huggingface_hub"):
        repo.list_files()
