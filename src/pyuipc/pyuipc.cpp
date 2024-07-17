#include "pybind11/pybind11.h"

// #include "export_diff_app.h"

namespace uipc::py {

int add(int a, int b) {
	return a + b;
}

}// namespace uipc::py

PYBIND11_MODULE(pyuipc, m) {
	m.doc() = "ing python binding";
	m.def("add", &uipc::py::add, "A function which adds two numbers");
	// sail::ing::py::export_diff_render_app(m);
}
