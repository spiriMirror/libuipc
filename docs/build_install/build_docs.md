# Build and Preview the Documentation

Download and install [doxygen](https://www.doxygen.nl/download.html).

Install [mkdocs](https://www.mkdocs.org/) and its plugins:

```shell
pip install mkdocs mkdocs-material mkdocs-literate-nav mkdoxy mkdocs-video
```

Build the same site that is deployed, including the generated C++ API pages:

```shell
python scripts/build_docs.py -o output/site
```

The build helper uses the active Python interpreter. On Windows it also detects
Doxygen in the default `Program Files/doxygen/bin` install directory when that
directory is not on `PATH`.

Serve the generated site locally:

```shell
python -m http.server 8000 --directory output/site
```

Open [http://127.0.0.1:8000/](http://127.0.0.1:8000/). Stop the server with
`Ctrl+C`.

For live reload while editing prose, use:

```shell
python -m mkdocs serve -f mkdocs-with-api.yaml
```

This command requires `doxygen` to be on `PATH`. A faster prose-only preview is
available with `python -m mkdocs serve -f mkdocs.yaml`, but that configuration
does not generate the `Libuipc/` API tree. API links returning 404 in a
prose-only preview are therefore expected; always validate the deployable site
with `scripts/build_docs.py` before publishing.
