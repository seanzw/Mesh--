#ifndef FACE_HEADER
#define FACE_HEADER

#include "Edge.h"

namespace mmm {
    class Face {
    public:
        Face(int v1, int v2, int v3) : v1(v1), v2(v2), v3(v3) {};
        ~Face() {}

        // Get the other two vertexes against v.
        Edge against(int v) const;

        // Reverse the orientation of the face.
        void reverse();

        // Replace v with vNew;
        void replace(int v, int vNew);

        int v1, v2, v3;
    };
}

#endif