"""Asset management for retrieving IPC research datasets, assets, and papers from Hugging Face.

This module provides a Python API to list, query, and download resources
(datasets, meshes, simulation configs, papers, etc.) hosted on a Hugging Face
repository.  It is the in-repo interface described in the project plan:

1. Data is collected and uploaded to a Hugging Face repository.
2. **This module** provides the programmatic interface.
3. Users call the API to retrieve data from Hugging Face.
4. A CI pipeline can then profile and analyse downloaded assets automatically.

Quick start::

    from uipc.asset import AssetRepo

    repo = AssetRepo()                # uses the default HF repo
    files = repo.list_files()         # list every file in the repo
    path  = repo.download("mesh/cube.obj")  # download and return local path

The ``huggingface_hub`` package is an **optional** dependency.  Import errors
are deferred until an operation that actually requires network access so that
the rest of *uipc* keeps working even when the hub library is absent.
"""

from __future__ import annotations

import os
import pathlib
import fnmatch
from typing import Dict, List, Optional

__all__ = [
    "DEFAULT_REPO_ID",
    "AssetRepo",
]

DEFAULT_REPO_ID: str = "spiriMirror/libuipc-data"
"""Default Hugging Face repository that hosts IPC assets."""

DEFAULT_REPO_TYPE: str = "dataset"
"""Default Hugging Face repository type."""


def _require_huggingface_hub():
    """Import and return the *huggingface_hub* module, raising a clear error
    if it is not installed."""
    try:
        import huggingface_hub
        return huggingface_hub
    except ImportError:
        raise ImportError(
            "The 'huggingface_hub' package is required for asset management. "
            "Install it with:  pip install huggingface_hub"
        ) from None


class AssetRepo:
    """Interface for listing, querying, and downloading assets from a
    Hugging Face repository.

    Parameters
    ----------
    repo_id : str, optional
        Hugging Face repository identifier (``"org/repo"``).
        Defaults to :data:`DEFAULT_REPO_ID`.
    repo_type : str, optional
        Hugging Face repository type (``"dataset"``, ``"model"``, or ``"space"``).
        Defaults to :data:`DEFAULT_REPO_TYPE`.
    cache_dir : str or None, optional
        Local directory used to cache downloaded files.  When *None* the
        ``huggingface_hub`` default cache location is used.
    token : str or None, optional
        Hugging Face API token for private repositories.  When *None* the
        token from the environment / ``huggingface-cli login`` is used.
    revision : str or None, optional
        Git revision (branch, tag, or commit SHA) to use.  *None* means the
        default branch.
    """

    def __init__(
        self,
        repo_id: str = DEFAULT_REPO_ID,
        repo_type: str = DEFAULT_REPO_TYPE,
        cache_dir: Optional[str] = None,
        token: Optional[str] = None,
        revision: Optional[str] = None,
    ) -> None:
        self._repo_id = repo_id
        self._repo_type = repo_type
        self._cache_dir = cache_dir
        self._token = token
        self._revision = revision

    # -- properties ----------------------------------------------------------

    @property
    def repo_id(self) -> str:
        """Hugging Face repository identifier."""
        return self._repo_id

    @property
    def repo_type(self) -> str:
        """Hugging Face repository type."""
        return self._repo_type

    @property
    def revision(self) -> Optional[str]:
        """Git revision used for this repository."""
        return self._revision

    # -- public API ----------------------------------------------------------

    def list_files(self, pattern: Optional[str] = None) -> List[str]:
        """Return file paths available in the repository.

        Parameters
        ----------
        pattern : str or None, optional
            A glob-style pattern (e.g. ``"*.obj"`` or ``"meshes/*.tet"``).
            When *None* every file is returned.

        Returns
        -------
        list of str
            Sorted list of matching file paths (relative to repo root).
        """
        hub = _require_huggingface_hub()
        api = hub.HfApi(token=self._token)
        all_files = api.list_repo_files(
            repo_id=self._repo_id,
            repo_type=self._repo_type,
            revision=self._revision,
        )
        if pattern is not None:
            all_files = [f for f in all_files if fnmatch.fnmatch(f, pattern)]
        return sorted(all_files)

    def download(
        self,
        filename: str,
        local_dir: Optional[str] = None,
    ) -> str:
        """Download a single file from the repository.

        Parameters
        ----------
        filename : str
            Path of the file inside the repository (e.g. ``"mesh/cube.obj"``).
        local_dir : str or None, optional
            Directory to download the file into.  When *None* the hub cache
            is used and a symlink / pointer path is returned.

        Returns
        -------
        str
            Absolute local path of the downloaded file.
        """
        hub = _require_huggingface_hub()
        path = hub.hf_hub_download(
            repo_id=self._repo_id,
            filename=filename,
            repo_type=self._repo_type,
            cache_dir=self._cache_dir,
            local_dir=local_dir,
            token=self._token,
            revision=self._revision,
        )
        return str(pathlib.Path(path).resolve())

    def download_files(
        self,
        pattern: str,
        local_dir: Optional[str] = None,
    ) -> List[str]:
        """Download every file that matches *pattern*.

        Parameters
        ----------
        pattern : str
            A glob-style pattern (e.g. ``"meshes/*.obj"``).
        local_dir : str or None, optional
            Target directory.  Behaves the same as in :meth:`download`.

        Returns
        -------
        list of str
            Sorted list of absolute local paths of downloaded files.
        """
        matching = self.list_files(pattern=pattern)
        return sorted(self.download(f, local_dir=local_dir) for f in matching)

    def file_info(self, filename: str) -> Dict:
        """Return metadata for a single file in the repository.

        The returned dictionary contains at least ``"filename"``,
        ``"size"`` (bytes), and ``"lfs"`` (LFS metadata or *None*).

        Parameters
        ----------
        filename : str
            Path of the file inside the repository.

        Returns
        -------
        dict
            File metadata.
        """
        hub = _require_huggingface_hub()
        api = hub.HfApi(token=self._token)
        repo_info = api.repo_info(
            repo_id=self._repo_id,
            repo_type=self._repo_type,
            revision=self._revision,
        )
        for sibling in repo_info.siblings:
            if sibling.rfilename == filename:
                return {
                    "filename": sibling.rfilename,
                    "size": sibling.size,
                    "lfs": sibling.lfs,
                }
        raise FileNotFoundError(
            f"File {filename!r} not found in repo {self._repo_id!r}"
        )

    def __repr__(self) -> str:
        parts = [f"repo_id={self._repo_id!r}"]
        if self._repo_type != DEFAULT_REPO_TYPE:
            parts.append(f"repo_type={self._repo_type!r}")
        if self._revision is not None:
            parts.append(f"revision={self._revision!r}")
        return f"AssetRepo({', '.join(parts)})"
