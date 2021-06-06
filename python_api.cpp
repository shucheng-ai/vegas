#include "vegas.h"

namespace vegas {

    py::list components (Document const &doc, double relax) {
        vector<Box> ccs;
        extract_cc(doc, &ccs, relax);
        py::list l;
        for (auto const &b: ccs) {
            l.append(b.unpack());
        }
        return l;
    }
}



PYBIND11_MODULE(vegas_core, module)
{
    using namespace vegas;
    module.doc() = "";
    py::class_<Document>(module, "Document")
        .def(py::init<>())
        .def("load", &DocumentLoader::load)
        ;
    py::class_<DocumentLoader>(module, "DocumentLoader")
        .def(py::init<>())
        .def("select", &DocumentLoader::select)
        .def("add", &DocumentLoader::add)
        .def("save", &DocumentLoader::save)
        ;
    py::class_<Box>(module, "Box")
        .def(py::init<>())
        .def("extend", &Box::extend_list)
        .def("extend", &Box::extend_tuple)
        .def("unpack", &Box::unpack)
        ;
    module.def("bound", (Box (*)(Document const &))&bound);
    module.def("components", &components);
}

