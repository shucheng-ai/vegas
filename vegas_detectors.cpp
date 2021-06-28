#include "vegas.h"

namespace vegas { namespace detectors {
    class simple_box: public Detector {
        double relax;
    public:
        simple_box (py::dict conf) {
            relax = conf["relax"].cast<double>();
        }

        virtual void detect (View const &view, Detection *det) const {
            view.collect_markup_objects(det);
            vector<Line> lines;
            view.collect(&lines);
            // detect
            ConnectedComponents cc(relax);
            for (auto const &l: lines) {
                cc.add(l);
            }
            cc.cleanup();
            vector<Box> boxes;
            cc.swap(&boxes);
            for (auto const &box: boxes) {
                det->objects.emplace_back();
                auto &obj = det->objects.back();
                obj.label = view.label;
                obj.bbox = box;
                box.contour(&obj.contour);
            }
        }
    };
    REGISTER(simple_box);
}}

