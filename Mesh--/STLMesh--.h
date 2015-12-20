#ifndef STL_MESH_MINUS_MINUS_HEADER
#define STL_MESH_MINUS_MINUS_HEADER

#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <queue>
#include <cassert>

#include "Mesh--.h"

#define BUFFER_SIZE 1024
#define INFD 1e8
#define EPSLOOSE 0.1
#define EPS 1e-8
#define TOLERATE 2.0
using std::min;
using std::max;
using std::make_pair;

namespace mmm {
    class STLMesh : public Mesh {

    public:

        STLMesh(std::istream &in, double threshold);
        virtual ~STLMesh();

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

        class Vector {
            double a[4];
            int s;
        public:
            Vector() {
                s = 0;
            }
            Vector(int n) {
                s = n;
            }
            Vector(double x, double y, double z) {
                s = 3;
                a[0] = x;
                a[1] = y;
                a[2] = z;
            }
            Vector(int n, double x) {
                s = n;
                for (int i = 0; i < n; i++)
                    a[i] = x;
            }
            void swap(Vector &b) {
                for (int i = 0; i < s; i++)
                    std::swap(a[i], b.a[i]);
            }
            int size() const { return s; }
            void clear() { s = 0; }
            double& operator [] (int x) { return a[x]; }
            const double& operator [] (int x) const { return a[x]; }
            void push_back(double x) { a[s++] = x; }
            void pop_back() { s--; }
        };
        typedef std::vector<Vector> Matrix;

        struct Edge {
            int v1;
            int v2;
            Edge(int v1 = -1, int v2 = -1) : v1(v1), v2(v2) {}

            bool operator < (const Edge &e2) const {
                if (v1 < e2.v1) {
                    return true;
                }
                else if (v1 == e2.v1) {
                    return v2 < e2.v2;
                }
                else {
                    return false;
                }
            }
        };


        void printVector(const Vector &v) {
            for (int i = 0; i < v.size(); i++)
                printf("%.4lf\t", v[i]);
            printf("\n");
        }

        void printMatrx(const Matrix &m) {
            for (auto v : m)
                printVector(v);
        }

        double norm(const Vector &v) {
            double t = 0;
            for (int i = 0; i < v.size(); i++) t += v[i] * v[i];
            return sqrt(t);
        }

        Vector crossProduct(const Vector &a, const Vector &b) {
            assert(a.size() == 3 && b.size() == 3);
            Vector c(3);
            c[0] = a[1] * b[2] - a[2] * b[1];
            c[1] = a[2] * b[0] - a[0] * b[2];
            c[2] = a[0] * b[1] - a[1] * b[0];
            return c;
        }

        double innerProduct(const Vector &a, const Vector &b) {
            assert(a.size() == b.size());
            double c = 0;
            for (int i = 0; i < a.size(); i++)
                c += a[i] * b[i];
            return c;
        }

        Matrix outerProduct(const Vector &a, const Vector &b) {
            Matrix c(a.size(), Vector(b.size(), 0));
            for (int i = 0; i < a.size(); i++)
                for (int j = 0; j < b.size(); j++)
                    c[i][j] = a[i] * b[j];
            return c;
        }

        /* Outer product accumulator. */
        void outerProductAcc(const Vector &a, const Vector &b, Matrix &acc) {
            assert(a.size() == acc.size());
            if (a.size() == 0) return;
            assert(b.size() == acc[0].size());
            for (int i = 0; i < a.size(); i++)
                for (int j = 0; j < b.size(); j++)
                    acc[i][j] += a[i] * b[j];
        }

        Vector innerProduct(const Vector &a, const Matrix &b) {
            assert(a.size() == b.size());
            if (a.size() == 0) return Vector();
            Vector c(b[0].size(), 0);
            for (int i = 0; i < b.size(); i++)
                for (int j = 0; j < b[0].size(); j++)
                    c[j] += a[i] * b[i][j];
            return c;
        }

        friend Vector operator + (const Vector &a, const Vector &b) {
            assert(a.size() == b.size());
            Vector c(a.size());
            for (int i = 0; i < a.size(); i++)
                c[i] = a[i] + b[i];
            return c;
        }

        friend Matrix operator + (const Matrix &a, const Matrix &b) {
            assert(a.size() == b.size());
            Matrix c(a.size());
            for (int i = 0; i < a.size(); i++)
                c[i] = a[i] + b[i];
            return c;
        }

        friend Vector operator - (const Vector &a, const Vector &b) {
            assert(a.size() == b.size());
            Vector c(a.size());
            for (int i = 0; i < a.size(); i++)
                c[i] = a[i] - b[i];
            return c;
        }

        friend Vector operator * (const double &a, const Vector &b) {
            Vector c(b.size());
            for (int i = 0; i < b.size(); i++)
                c[i] = a * b[i];
            return c;
        }

        friend Vector operator / (const Vector &a, const double &b) {
            assert(b != 0);
            Vector c(a.size());
            for (int i = 0; i < a.size(); i++)
                c[i] = a[i] / b;
            return c;
        }



        /**************************************************************/

        class CrossLink {
        public:
            CrossLink(int size) : s(size) {
                edgeX = new Node *[size];
                edgeY = new Node *[size];
                for (int i = 0; i < size; ++i) {
                    edgeX[i] = edgeY[i] = nullptr;
                }
            }

            ~CrossLink() {
                for (int i = 0; i < s; ++i) {
                    Node *node = edgeX[i];
                    while (node != nullptr) {
                        Node *next = node->dwn;
                        delete node;
                        node = next;
                    }
                }
                delete[] edgeX;
                delete[] edgeY;
            }

            inline int size() const { return s; }

            bool insert(int x, int y) {

                Node *p = edgeX[x];
                Node *node = new Node(x, y);
                if (p == nullptr) {
                    edgeX[x] = node;
                }
                else {
                    while (p != nullptr) {
                        if (p->y > y) {
                            node->dwn = edgeX[x];
                            edgeX[x]->ups = node;
                            edgeX[x] = node;
                            break;
                        }
                        else if (p->y == y) {
                            delete node;
                            return true;
                        }
                        else {
                            if (p->dwn == nullptr) {
                                p->dwn = node;
                                node->ups = p;
                                break;
                            }
                            else if (p->dwn->y > y) {
                                node->dwn = p->dwn;
                                node->ups = p;
                                p->dwn->ups = node;
                                p->dwn = node;
                                break;
                            }
                            else {
                                p = p->dwn;
                            }
                        }
                    }
                }
                p = edgeY[y];
                if (p == nullptr) {
                    edgeY[y] = node;
                }
                else {
                    while (p != nullptr) {
                        if (p->x > x) {
                            node->rhs = edgeY[y];
                            edgeY[y]->lhs = node;
                            edgeY[y] = node;
                            break;
                        }
                        else if (p->x == x) {
                            return true;
                        }
                        else {
                            if (p->rhs == nullptr) {
                                p->rhs = node;
                                node->lhs = p;
                                break;
                            }
                            else if (p->rhs->x > x) {
                                node->rhs = p->rhs;
                                node->lhs = p;
                                p->rhs->lhs = node;
                                p->rhs = node;
                                break;
                            }
                            else {
                                p = p->rhs;
                            }
                        }
                    }
                }
                return false;
            }

            bool find(int x, int y) {
                Node *p = edgeX[x];
                if (p == nullptr) {
                    return false;
                }
                else {
                    while (p != nullptr) {
                        if (p->y > y) {
                            return false;
                        }
                        else if (p->y == y) {
                            return true;
                        }
                        else {
                            if (p->dwn == nullptr) {
                                return false;
                            }
                            else if (p->dwn->y > y) {
                                return false;
                            }
                            else {
                                p = p->dwn;
                            }
                        }
                    }
                }
                return false;
            }

            void erase(int x, int y) {
                Node *p = edgeX[x];
                while (p != nullptr) {
                    if (p->y > y) {
                        break;
                    }
                    else if (p->y == y) {

                        if (p->lhs != nullptr) {
                            p->lhs->rhs = p->rhs;
                        }
                        else {
                            assert(p == edgeY[y]);
                            edgeY[y] = p->rhs;
                        }

                        if (p->rhs != nullptr) {
                            p->rhs->lhs = p->lhs;
                        }

                        if (p->ups != nullptr) {
                            p->ups->dwn = p->dwn;
                        }
                        else {
                            assert(p == edgeX[x]);
                            edgeX[x] = p->dwn;
                        }

                        if (p->dwn != nullptr) {
                            p->dwn->ups = p->ups;
                        }

                        delete p;
                        break;
                    }
                    else {
                        if (p->dwn == nullptr) {
                            break;
                        }
                        else if (p->dwn->y > y) {
                            break;
                        }
                        else {
                            p = p->dwn;
                        }
                    }
                } 
            }

            struct Node {
                Node *lhs, *rhs, *ups, *dwn;
                int x;
                int y;
                Node(int x, int y) : x(x), y(y) {
                    lhs = rhs = ups = dwn = nullptr;
                }
            };
            int s;
            Node **edgeX;
            Node **edgeY;
        };

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