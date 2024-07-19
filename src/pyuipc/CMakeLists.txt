find_package (Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
# Create an extension module
pybind11_add_module(pyuipc pyuipc.cpp)
target_compile_features(pyuipc PUBLIC cxx_std_20)
target_link_libraries(pyuipc PUBLIC pybind11::module)
target_link_libraries(pyuipc PUBLIC uipc::uipc)
uipc_set_output_directory(pyuipc)