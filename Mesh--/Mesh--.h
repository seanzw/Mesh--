#ifndef MESH_MINUS_MINUS_HEADER
#define MESH_MINUS_MINUS_HEADER

#include <iostream>

namespace mmm {
    class Mesh {
    public:
        Mesh() {}
        virtual ~Mesh() {}

        /* Output Obj file. */
        virtual void dumpObj(std::ostream &out) const = 0;

        /**
        * Simplify the mesh with edge collapse.
        */
        virtual void simplify(size_t remain_verts) = 0;

        virtual inline size_t getNumFaces() const = 0;
        virtual inline size_t getNumEdges() const = 0;
        virtual inline size_t getNumVerts() const = 0;

        virtual inline size_t getOldNumFaces() const = 0;
        virtual inline size_t getOldNumEdges() const = 0;
        virtual inline size_t getOldNumVerts() const = 0;
    };
};

#endif
