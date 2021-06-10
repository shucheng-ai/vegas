#include <memory>
#include <09_numpy_cpp-custom-matrix/matrix.h>
#include <09_numpy_cpp-custom-matrix/pybind_matrix.h>
#include "vegas.h"

namespace vegas {
    using std::unique_ptr;

    py::list components (Document const &doc, double relax) {
        vector<Box> ccs;
        extract_cc(doc, &ccs, relax);
        py::list l;
        for (auto const &b: ccs) {
            l.append(b.unpack());
        }
        return l;
    }

    class PyExtractor {
        vector<unique_ptr<Extractor>> xtors;
    public:
        PyExtractor (py::list confs) {
            for (auto h: confs) {
                xtors.emplace_back(Extractor::create(h.cast<py::dict>()));
            }
        }

        Matrix<double> apply (Document const &doc) const {
            unsigned long layers = doc.layers.size();
            unsigned long dims = 0;
            for (auto const &p: xtors) {
                dims += p->dim();
            }
            Matrix<double> fts({layers, dims});
            for (int i = 0; i < layers; ++i) {
                double *begin = &fts(i);
                double *off = begin;
                for (auto const &p: xtors) {
                    p->apply(doc.layers[i], off);
                    off += p->dim();
                }
                CHECK(off - begin == dims);
            }
            return fts;
        }
    };
}



PYBIND11_MODULE(vegas_core, module)
{
    using namespace vegas;
    module.doc() = "";
    py::class_<Document>(module, "Document")
        .def(py::init<>())
        .def("load", &DocumentLoader::load)
        .def("size", &DocumentLoader::size)
        .def("render", &DocumentLoader::render)
        .def("layerName", &DocumentLoader::layerName)
        ;
    py::class_<Annotation>(module, "Annotation")
        .def(py::init<>())
        .def("load", &Annotation::load_json)
        .def("save", &Annotation::save_json)
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
    py::class_<PyExtractor>(module, "Extractor")
        .def(py::init<py::list>())
        .def("apply", &PyExtractor::apply)
        ;
    module.def("bound", (Box (*)(Document const &))&bound);
    module.def("components", &components);
}

