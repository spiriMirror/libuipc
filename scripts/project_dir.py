import pathlib as pl

def project_dir(): 
    # get this file's parent directory, don't use __file__ as it is not reliable
    this_file = pl.Path(__file__).resolve()
    return this_file.parent.parent

def backend_source_dir(backend_name:str):
    return project_dir() / 'src' / 'backends' / backend_name