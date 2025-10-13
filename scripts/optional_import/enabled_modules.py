class EnabledModules:
    modules: set[str] = set()

    def insert( name:str):
        EnabledModules.modules.add(name)
    
    def report():
        print(f'Enabled optional modules: {EnabledModules.modules}')
