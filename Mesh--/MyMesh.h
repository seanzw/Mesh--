#ifndef MY_MESH_HEADER
#define MY_MESH_HEADER

#include <iostream>
#include <vector>
#include <memory>
#include "glm\glm.hpp"

namespace mm {
    struct Vert {
        Vert(double x, double y, double z) : v(x, y, z), isDelted(false) {}

        glm::vec3 v;
        bool isDelted;
    };

    struct Face {
        
        int v1;
        int v2;
        int v3;
        glm::vec4 p;

        Face(int v1, int v2, int v3) : v1(v1), v2(v2), v3(v3) {}
        
        void computeP(const Vert &v1, const Vert &v2, const Vert &v3) {

            glm::vec3 normal = glm::normalize(glm::cross(v2.v - v1.v, v3.v - v1.v));
            double d = -dot(normal, v1.v);
            p = glm::vec4(normal, d);

        }
    };

    struct Edge {

        Edge(int vert1 = -1, int vert2 = -1) : split(0.0f) {

            if (vert1 < vert2) {
                v1 = vert1;
                v2 = vert2;
            }
            else {
                v1 = vert2;
                v2 = vert1;
            }

            lhs = rhs = ups = dwn = nullptr;
            heapIdx = -1;
            length = 0.0f;
        }
        
        double compute(const Vert &v1, const Vert &v2, const glm::mat4 &q1, const glm::mat4 &q2) {

            glm::mat4 q = q1 + q2;

            split = (v1.v + v2.v) * 0.5f;
            return dot(glm::vec4(split, 1.0f), q * glm::vec4(split, 1.0f));

        }

        int v1;
        int v2;
        int heapIdx;
        double length;
        Edge *lhs, *rhs, *ups, *dwn;
        glm::vec3 split;
    };

    class Mesh {
    public:
        Mesh(std::istream &in, double ratio, double maxDistance);
        ~Mesh();

        void dumpObj(std::ostream &out) const;

        void simplify();

        inline int getOriginalNumVerts() const { return num_verts; }
        inline int getOriginalNumFaces() const { return num_faces; }
        inline int getNewNumVerts() const { return num_verts - cut_verts; }
        inline int getNewNumFaces() const { return num_faces - cut_faces; }

    private:

        std::vector<Vert> verts;
        std::vector<Face> faces;

        std::vector<int> *vfTable;

        glm::mat4 *qVert;
        glm::mat4 *qFace;

        Edge **edgeX;
        Edge **edgeY;

        int num_verts;
        int num_faces;

        int cut_verts;
        int cut_faces;

        double ratio;
        double maxDistance;
        double ave_len;

        struct Pair {
            Pair(double cost, Edge *edge = nullptr) : cost(cost), edge(edge) {}

            double cost;
            Edge *edge;
        };

        std::vector<Pair> heap;

        /* Precomputation. */
        void precompute();

        /* Helper function to insert an edge into the cross link. */
        inline bool insertEdge(Edge *e);

        /* Helper function to pop heap. */
        inline void popHeap(int id);

        /* Helper function to compare if two faces share all the same verts. */
        inline bool compareFaces(const Face &f1, const Face &f2) const;

        /* Helper function to swap two pairs in the heap. */
        inline void swap(Pair &pair1, Pair &pair2) {
            Pair temp = pair1;
            pair1 = pair2;
            pair2 = temp;
        }

        /* Helper function to restore the heap. */
        void restoreHeap(int id);

        /* Build the heap. */
        void buildHeap();

        /* Collapse an edge, returning the number of faces cut. */
        int collapse();

    };
}


#endif