#include "vegas.h"

namespace vegas { namespace detectors {

    class wda_rooms: public Detector {
    public:
        wda_rooms (py::dict conf) {
        }

        virtual void detect (Layer const &layer, vector<Object> *) const {
        }
    };

    REGISTER(wda_rooms);
}}
