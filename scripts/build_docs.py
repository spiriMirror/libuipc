import subprocess as sp
import pathlib as pl

# get rpoject source dir
file = __file__
project_source_dir = pl.Path(file).parent.parent.absolute()
print(f"project_source_dir: {project_source_dir}")

# get the mkdocs config file
mkdocs_config_file = project_source_dir / "mkdocs.yaml"

# get the cwd
cwd = pl.Path.cwd().absolute()
print(f"cwd: {cwd}")

# build the docs
docs_dir = cwd / "docs"
sp.run(["mkdocs", "build", "-f", mkdocs_config_file, "-d", docs_dir], cwd=cwd)



