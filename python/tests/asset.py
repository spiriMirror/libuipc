import os 
import pathlib

class AssetDir:
    this_file = pathlib.Path(os.path.dirname(__file__)).resolve()
    _output_path = pathlib.Path(this_file / '../../output/').resolve()
    _assets_path = pathlib.Path(this_file / '../../assets/').resolve()
    _tetmesh_path = _assets_path / 'sim_data' / 'tetmesh'

    @staticmethod
    def asset_path():
        return str(AssetDir._assets_path)

    @staticmethod
    def tetmesh_path():
        return str(AssetDir._tetmesh_path)
    
    @staticmethod
    def output_path(file):
        file_dir = pathlib.Path(file).absolute()
        this_python_root = AssetDir.this_file.parent.parent
        # get the relative path from the python root to the file
        relative_path = file_dir.relative_to(this_python_root)
        # construct the output path
        output_dir = AssetDir._output_path / relative_path / ''
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        return str(output_dir)

    @staticmethod
    def folder(file):
        return pathlib.Path(file).absolute().parent
