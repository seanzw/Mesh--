#ifndef STL_MESH_MINUS_MINUS_HEADER
#define STL_MESH_MINUS_MINUS_HEADER

#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <queue>
#include <cassert>
#include <iostream>

#include "Vector.h"
#include "Edge.h"
#include "CrossLink.h"

#define BUFFER_SIZE 1024
#define INFD 1e8
#define EPSLOOSE 0.1
#define EPS 1e-8
#define TOLERATE 2.0
using std::min;
using std::max;
using std::make_pair;

namespace mmm {
    class Mesh {

    public:

        Mesh(std::istream &in, double threshold);
        virtual ~Mesh();

        /* Output Obj file. */
        virtual void dumpObj(std::ostream &out) const;

        /**
        * Simplify the mesh with edge collapse.
        */
        virtual void simplify(size_t remain_verts);

        virtual inline size_t getNumFaces() const { return num_faces; }
        virtual inline size_t getNumEdges() const { return edge->size(); }
        virtual inline size_t getNumVerts() const { return vert.size(); }

        virtual inline size_t getOldNumFaces() const { return old_num_faces; }
        virtual inline size_t getOldNumEdges() const { return old_num_edges; }
        virtual inline size_t getOldNumVerts() const { return old_num_verts; }

    private:
        
        /**************************************************************/

        std::vector<Vector> vert;           /* Vector for position. */
        std::vector<bool> removed;          /* Is this vert removed?. */
        std::vector< std::set<Edge> > face; /* Vert to edge table. */
        CrossLink *edge;

        /* Heap is a priority queue for Edge. */
        std::priority_queue< std::pair<double, Edge> > heap;

        size_t num_faces;                   /* # faces. */
        size_t old_num_faces;
        size_t old_num_verts;
        size_t old_num_edges;

        const double threshold;

        double edgeLen(const Edge &e) {
            return norm(vert[e.v1] - vert[e.v2]);
        }

        /* Get the split position of the edge. */
        std::pair<Vector, double> getPosition(const Edge &e);

        /* Select an edge to collapse. */
        std::pair<Edge, Vector> selectEdge(double threshold);

        /**
        * Check if after replacing v1 with v2, the normal of the triangle will be on the opposite direction.
        * The triangle looks like:
        *
        *      v1                  v2
        *     /  \                /  \
        *    /    \  ------>     /    \
        *   x ---- y            x ---- y
        *
        * @param e: the opposite edge.
        * @param v1: the old vert.
        * @param v2: the new vert.
        * @return bool: true if the face's normal changed, false otherwise.
        *
        */
        bool faceReverse(const Edge &e, const Vector &v1, const Vector &v2);

        /* Add an edge to the heap. */
        void addToHeap(const Edge &e, double threshold);

        /* Update the neighbor edge and add them to the heap. */
        void updateNeighborEdge(int v, double threshold);

        /**
         * Remove an edge from the mesh.
         * @param e: edge to be removed.
         * @param v: new position for v1.
         * @param threshold: maximum length for a removable edge.
         * @return void.
         */
        void removeEdge(const Edge &e, const Vector &v, double threshold);

        /* Push all the valid edges into the heap. */
        void buildHeap(double threshold);
    };
};

#endif