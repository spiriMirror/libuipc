from setuptools import setup
try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel
    class bdist_wheel(_bdist_wheel):
        def finalize_options(self):
            super().finalize_options()
            self.root_is_pure = False
except ImportError:
    bdist_wheel = None

setup(
    # to build the wheel with the correct tag (including the platform)
    # rather than the default tag (py3-none-any)
    cmdclass={'bdist_wheel': bdist_wheel},
)