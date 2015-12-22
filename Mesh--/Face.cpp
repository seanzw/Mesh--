
#include <cassert>
#include "Face.h"

namespace mmm {
    Edge Face::against(int v) const {
        assert(v == v1 || v == v2 || v == v3);
        if (v == v1) {
            return Edge(v2, v3);
        }
        else if (v == v2) {
            return Edge(v3, v1);
        }
        else {
            return Edge(v1, v2);
        }
    }
}