#include "vegas.h"

namespace vegas { namespace detectors {
    class objects: public Detector {
    public:
        objects (py::dict conf) {
        }

        virtual void detect (Layer const &layer, vector<Object> *) const {
        }
    };
    REGISTER(objects);
}}

