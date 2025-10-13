class EnabledModules:
    modules: set[str] = set()

    @staticmethod
    def insert(name: str):
        EnabledModules.modules.add(name)

    @staticmethod
    def report():
        print(f'Enabled optional modules: {EnabledModules.modules}')
