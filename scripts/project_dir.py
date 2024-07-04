import pathlib as pl


    
def project_dir(): 
    # get this file's parent directory, don't use __file__ as it is not reliable
    this_file = pl.Path(__file__).resolve()
    print(f"project_dir: {this_file.parent.parent}")
    return this_file.parent.parent