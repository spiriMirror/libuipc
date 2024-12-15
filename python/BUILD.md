# pyuipc

The Python IPC Module

Prerequist useful tools:

- `setuptools, build`: for packaging
- `mypy`: for type checking
- `pytest`: for testing framework
- `numpy`: package dependencies

`pip install setuptools build mypy pytest numpy`

## Development

- development install: `pip install -e .` install pyuipc package in editable mode, then you can dynamically change code in `src/pyuipc` and check them with `tests`
- run tests: `pytest`
- run specific example: `pytest tests/test_abd_fem.py`

## Package

When python module is stable, we would like to package it to wheels to support the downstream development.

- `python -m build`: will build as the configuration defined in `pyproject.toml`

