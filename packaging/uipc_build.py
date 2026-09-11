"""PEP 517 build backend for libuipc.

Every hook is forwarded to scikit-build-core, the CMake path that produces the
released wheels.  The single exception is an editable install that explicitly
asks for xmake:

    pip install -e . --config-settings=builder=xmake

Then the extension is built by xmake into ``python/src/uipc/_native`` (see the
``python_editable`` option) and setuptools turns ``python/`` into the editable
wheel.  Anything else -- ``build_wheel``, ``build_sdist``, a plain
``pip install -e .`` -- behaves exactly as if this module were not here.

Additional settings recognised in the xmake path::

    jobs=N            parallel compile jobs (default 4; nvcc OOMs on more)
    xmake-args="..."  extra flags appended to `xmake f`
"""

from __future__ import annotations

from contextlib import contextmanager
import os
import shutil
import subprocess
import sys
from pathlib import Path

# No __all__: PEP 517 frontends resolve hooks by attribute name, and listing
# the lazily forwarded ones there would only read as undefined.

# Hooks that belong entirely to scikit-build-core.  They are resolved lazily by
# __getattr__ below rather than imported here: with --no-build-isolation the
# xmake path runs in an environment that need not have scikit-build-core at all,
# and a module-level import would make the whole backend unimportable there.
_FORWARDED = frozenset(
    {
        "build_sdist",
        "build_wheel",
        "get_requires_for_build_sdist",
        "get_requires_for_build_wheel",
        "prepare_metadata_for_build_wheel",
    }
)


def _cmake_backend():
    # Provided by [build-system].requires, so a type checker aimed at a runtime
    # virtualenv will not resolve it.
    import scikit_build_core.build as backend  # type: ignore[import-not-found]

    return backend


def __getattr__(name: str):
    """Forward the CMake-only hooks to scikit-build-core on first access.

    PEP 517 frontends look hooks up as attributes, so resolving them here is
    equivalent to re-exporting them, minus the import cost when unused."""
    if name in _FORWARDED:
        return getattr(_cmake_backend(), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

_ROOT = Path(__file__).resolve().parent.parent
_PYTHON_DIR = _ROOT / "python"
_EDITABLE_VERSION = "0.9.0"


@contextmanager
def _editable_cmake_version():
    """Use the development version for editable CMake builds only.

    Release hooks continue to derive their version from Git.  The xmake
    editable path gets the same value from ``python/pyproject.toml``; setting
    the setuptools-scm override around the CMake hooks keeps both development
    backends consistent without changing published-wheel metadata.
    """
    variable = "SETUPTOOLS_SCM_PRETEND_VERSION"
    previous = os.environ.get(variable)
    os.environ[variable] = _EDITABLE_VERSION
    try:
        yield
    finally:
        if previous is None:
            os.environ.pop(variable, None)
        else:
            os.environ[variable] = previous


def _joined_setting(config_settings: dict | None, key: str) -> str:
    """Read a setting that carries several tokens.  pip hands us a list when the
    key is repeated; concatenate, the way a command line would accumulate."""
    value = (config_settings or {}).get(key, "")
    if isinstance(value, (list, tuple)):
        value = " ".join(str(item) for item in value)
    return str(value).strip()


def _last_setting(config_settings: dict | None, key: str) -> str:
    """Read a single-valued setting.  A repeated --config-settings=key=... acts
    like a repeated command line flag: the last one wins."""
    value = (config_settings or {}).get(key, "")
    if isinstance(value, (list, tuple)):
        value = value[-1] if value else ""
    return str(value).strip()


def _use_xmake(config_settings: dict | None) -> bool:
    builder = _last_setting(config_settings, "builder") or os.environ.get("UIPC_BUILDER", "")
    return builder.strip().lower() == "xmake"


def _cmake_config_settings(config_settings: dict | None) -> dict | None:
    """Remove settings owned by this shim before delegating to scikit-build-core."""
    if not config_settings:
        return config_settings
    return {
        key: value
        for key, value in config_settings.items()
        if key not in {"builder", "jobs", "xmake-args"}
    }


def _run_xmake(config_settings: dict | None) -> None:
    xmake = shutil.which("xmake")
    if xmake is None:
        raise RuntimeError(
            "builder=xmake was requested but xmake was not found on PATH; "
            "see https://xmake.io for installation instructions"
        )

    env = os.environ.copy()
    # xmake resolves the interpreter through `python3` on PATH.  Put the one
    # running this build first, so the extension is compiled against the ABI of
    # the environment it is about to be installed into.
    env["PATH"] = os.pathsep.join([str(Path(sys.executable).parent), env.get("PATH", "")])
    # `xmake f` resets every option that is not passed explicitly, so sharing a
    # configuration directory with the checkout would silently wipe the options
    # a developer set by hand.  Keep ours separate.  Object files still live in
    # build/, but only get reused when the options agree -- a checkout built as
    # releasedbg recompiles here, since we default to release.
    env["XMAKE_CONFIGDIR"] = str(_ROOT / "build" / "xmake-pep517")

    configure = [
        xmake,
        "f",
        "--pybind=true",
        "--python_editable=true",
        "--python_system=true",
        f"--python_version={sys.version_info.major}.{sys.version_info.minor}.x",
    ]
    # Escape hatch for anything else the caller needs, e.g.
    # --config-settings="xmake-args=-m releasedbg --backend_cuda=false".
    # Appended last so an explicit flag overrides the defaults above.
    extra = _joined_setting(config_settings, "xmake-args")
    if extra:
        configure += extra.split()

    subprocess.run(configure, cwd=_ROOT, env=env, check=True)

    # nvcc is memory-hungry, and xmake defaults to one job per core, which OOMs
    # on the CUDA backend.  Cap at 4, overridable via --config-settings=jobs=N.
    jobs = _last_setting(config_settings, "jobs") or os.environ.get("UIPC_BUILD_JOBS", "") or "4"
    subprocess.run([xmake, "build", "-j", jobs, "pyuipc"], cwd=_ROOT, env=env, check=True)


def _setuptools_hook(hook: str, *args: object) -> str:
    """Run a setuptools hook against python/, which carries its own metadata and
    a plain setuptools backend.  config_settings is deliberately not forwarded:
    ours holds keys (builder, xmake-args) that setuptools would reject."""
    try:
        import setuptools.build_meta as backend  # type: ignore[import-not-found]
    except ModuleNotFoundError as error:  # pragma: no cover - environment issue
        # With --no-build-isolation pip ignores [build-system].requires, so a
        # bare environment reaches here with nothing to build the wheel with.
        raise RuntimeError(
            "builder=xmake needs setuptools in the current environment when "
            "--no-build-isolation is used; run `pip install setuptools` first, "
            "or drop --no-build-isolation and let pip provision it"
        ) from error

    cwd = Path.cwd()
    os.chdir(_PYTHON_DIR)
    try:
        return getattr(backend, hook)(*args)
    finally:
        os.chdir(cwd)


def get_requires_for_build_editable(config_settings: dict | None = None) -> list[str]:
    if _use_xmake(config_settings):
        # setuptools and wheel are already pinned in [build-system].requires,
        # and xmake drives the compiler itself -- nothing to add.
        return []
    with _editable_cmake_version():
        return _cmake_backend().get_requires_for_build_editable(
            _cmake_config_settings(config_settings)
        )


def prepare_metadata_for_build_editable(
    metadata_directory: str, config_settings: dict | None = None
) -> str:
    if _use_xmake(config_settings):
        return _setuptools_hook("prepare_metadata_for_build_editable", metadata_directory, None)
    with _editable_cmake_version():
        return _cmake_backend().prepare_metadata_for_build_editable(
            metadata_directory, _cmake_config_settings(config_settings)
        )


def build_editable(
    wheel_directory: str,
    config_settings: dict | None = None,
    metadata_directory: str | None = None,
) -> str:
    if not _use_xmake(config_settings):
        with _editable_cmake_version():
            return _cmake_backend().build_editable(
                wheel_directory,
                _cmake_config_settings(config_settings),
                metadata_directory,
            )
    _run_xmake(config_settings)
    return _setuptools_hook("build_editable", wheel_directory, None, metadata_directory)
