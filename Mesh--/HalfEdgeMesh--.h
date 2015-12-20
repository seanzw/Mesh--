#ifndef HALF_EDGE_MESH_MINUS_MINUS_HEADER
#define HALF_EDGE_MESH_MINUS_MINUS_HEADER

#include <utility>
#include <algorithm>
#include <map>
#include <iterator>
#include <vector>

#include "glm\glm.hpp"
#include "Mesh--.h"
//#define DEBUG

namespace mmm {

    typedef glm::vec3 Vec3;

    class HalfEdgeMesh : public Mesh {
    public:
        HalfEdgeMesh(std::istream &in);
        ~HalfEdgeMesh();

        /* Output Obj file. */
        virtual void dumpObj(std::ostream &out) const;

        // Used for debug.
        void dumpDebug() const;

        /**
         * Simplify the mesh with edge collapse.
         */
        virtual void simplify(size_t remain_verts);

        /* API. */
        virtual inline size_t getNumFaces() const { return num_faces; }
        virtual inline size_t getNumEdges() const { return num_edges; }
        virtual inline size_t getNumVerts() const { return num_verts; }

        virtual inline size_t getOldNumFaces() const { return old_num_faces; }
        virtual inline size_t getOldNumEdges() const { return old_num_edges; }
        virtual inline size_t getOldNumVerts() const { return old_num_verts; }

    private:

        class Vert;
        class Face;

        class Edge {
        public:

            /* Universal id. */
            const size_t id;

            Vert *vert;     // Vertex at the end of the half-edge.
            Edge *pair;     // Oppositely oriented adjacent half-edge.
            Face *face;     // Face beside the half edge.
            Edge *next;     // Next half-edge around the face.

            Edge *link;     // Pointer to next node in the list.
            glm::vec3 e;    // vec3 for this edge.
            double len;     // Length of the edge.
            bool is;        // Switch.

            int heapId;     // Index to the pair in the heap.

            Edge(const glm::vec3 &e,
                Vert *vert = nullptr,
                Edge *pair = nullptr,
                Face *face = nullptr,
                Edge *next = nullptr
                ) : id(createId()), is(true), e(e), vert(vert), pair(pair), face(face), next(next), link(nullptr) {
                len = glm::length(e);
                heapId = -1;
            }

            inline bool isBoundary() const {
                return pair == nullptr;
            }

            inline Vert *tgt() const {
                return vert;
            }

            inline Vert *src() const {
                return next->next->vert;
            }

            void computeE();
            void dump(std::ostream &out);

        private:
            static size_t createId() {
                static size_t id = 1;
                return id++;
            }
        };

        class Vert {
        public:

            /* Universal id. */
            const size_t id;
            glm::vec3 pos;

            Edge *edge;     // One of the half-edges leaving the vertex.
            Vert *link;     // Pointer to next node in the list;
            glm::mat4 Q;    // Cost matrix for this vert.
            bool is;        // Switch.

            Vert(const glm::vec3 &pos,
                Edge *edge = nullptr
                ) : id(createId()), pos(pos), edge(edge), link(nullptr), Q(0.0f), is(true) {}

            /* The degree (# Edges) of this vert. */
            size_t degree() const;

            inline bool isBoundary() const {
                Edge *edge = this->edge;
                do {
                    if (edge->pair == nullptr) {
                        return true;
                    }
                    else {
                        edge = edge->pair->next;
                    }
                } while (edge != this->edge);
                return false;
            }

            int selfCheck() const {
                Edge *e = edge;
                do {
                    Edge *e_ = edge;
                    while (e_ != e) {
                        assert(e_->vert->id != e->vert->id);
                        e_ = e_->pair->next;
                    }
                    if (e->pair == nullptr) {
                        break;
                    }
                    else {
                        assert(e->pair->vert->id == id);
                        assert(e->vert->id != id);
                        e = e->pair->next;
                    }
                } while (e != edge);
                return 0;
            }

            void dump(std::ostream &out);
        private:
            static size_t createId() {
                static size_t id = 1;
                return id++;
            }
        };

        class Face {
        public:

            /* Universal id. */
            const size_t id;

            Edge *edge;     // One of the half-edge besides the face.
            Face *link;     // Pointer to next node in the list.
            glm::mat4 Q;
            bool is;        // Switch.
            Face(const glm::mat4 &Q = glm::mat4(0.0f),
                Edge *edge = nullptr
                ) : id(createId()), edge(edge), link(nullptr), Q(Q), is(true) {}

            void computeQ() {
                glm::vec3 normal = glm::cross(edge->e, edge->next->e);
                normal = glm::normalize(normal);
                glm::vec4 p(normal, glm::dot(edge->vert->pos, normal));
                Q = glm::mat4(p * p.x, p * p.y, p * p.z, p * p.w);
            }

            double area() {
                Vec3 normal = glm::cross(edge->e, edge->next->e);
                return glm::length(normal);
            }

            void dump(std::ostream &out);
        private:
            static size_t createId() {
                static size_t id = 1;
                return id++;
            }
        };

        struct Pair {
            Edge *edge;
            glm::vec3 v;
            double cost;
        };

        class MinHeap {
        public:
            MinHeap(size_t capacity) : capacity(capacity), size(0) {
                buffer = new Pair[capacity];
            }
            ~MinHeap() {
                delete[] buffer;
            }

            void emplace_back(Edge *edge, const glm::vec3 &v, double cost) {
                buffer[size].edge = edge;
                buffer[size].v = v;
                buffer[size].cost = cost;
                edge->heapId = (int)size;
                if (edge->pair != nullptr) {
                    edge->pair->heapId = (int)size;
                }
                size++;
                restore(size - 1);
            }

            void update(size_t id, const glm::vec3 &v, double cost) {
                buffer[id].cost = cost;
                buffer[id].v = v;
                restore(id);
            }

            Pair pop(size_t id) {
                Pair ret = buffer[id];
                swap(id, size - 1);
                assert(ret.edge->heapId == size - 1);
                ret.edge->heapId = -1;
                if (ret.edge->pair != nullptr) {
                    assert(ret.edge->pair->heapId == size - 1);
                    ret.edge->pair->heapId = -1;
                }
                size--;
                restore(id);
                return ret;
            }
            size_t size;

            bool selfCheck(std::ostream &out, double maxLength) const;

            void dump(std::ostream &out) const;

        private:
            size_t capacity;
            Pair *buffer;

            inline void swap(size_t i, size_t j) {
                auto temp = buffer[i];
                buffer[i] = buffer[j];
                buffer[j] = temp;
                buffer[i].edge->heapId = (int)i;
                buffer[j].edge->heapId = (int)j;
                if (buffer[i].edge->pair != nullptr)
                    buffer[i].edge->pair->heapId = (int)i;
                if (buffer[j].edge->pair != nullptr)
                    buffer[j].edge->pair->heapId = (int)j;
            }

            void restore(size_t id) {
                up(id);
                down(id);
            }

            void up(size_t id) {
                size_t parent = (id - 1) / 2;
                while (id > 0 && buffer[id].cost < buffer[parent].cost) {
                    swap(id, parent);
                    id = parent;
                    parent = (id - 1) / 2;
                }
            }

            void down(size_t id) {
                size_t lhs = (id * 2) + 1;
                size_t rhs = (id * 2) + 2;
                while (lhs < size) {
                    if (rhs < size) {
                        if (buffer[lhs].cost < buffer[rhs].cost &&
                            buffer[lhs].cost < buffer[id].cost) {
                            swap(lhs, id);
                            id = lhs;
                            lhs = (id * 2 + 1);
                            rhs = (id * 2 + 2);
                        }
                        else if (buffer[rhs].cost < buffer[lhs].cost &&
                            buffer[rhs].cost < buffer[id].cost) {
                            swap(rhs, id);
                            id = rhs;
                            lhs = (id * 2 + 1);
                            rhs = (id * 2 + 2);
                        }
                        else {
                            break;
                        }
                    }
                    else {
                        if (buffer[lhs].cost < buffer[id].cost) {
                            swap(lhs, id);
                        }
                        break;
                    }
                }
            }
        };

        Vert *verts;
        Face *faces;
        Edge *edges;

        /* Current information. */
        size_t num_verts;
        size_t num_faces;
        size_t num_edges;

        /* Old information. */
        size_t old_num_verts;
        size_t old_num_faces;
        size_t old_num_edges;

        /* Max length. */
        double maxLength;

        std::map<std::pair<Vert *, Vert *>, Edge *> edgeHashMap;

        void buildHeap(MinHeap &heap, double maxLength);

        int collapse(const Pair &pair, MinHeap &heap);

        inline Edge *make_edge(Vert *v1, Vert *v2, Face *face) {
            Edge *ret;
            // New edge.
            ret = new Edge(v2->pos - v1->pos, v2);
            ret->link = edges;
            edges = ret;
            v1->edge = ret;
            ret->face = face;
            return ret;
        }

        /* Is this edge legal to collapse. */
        static int isEdgeNotCollapsable(Edge *e, double maxLength) {

            assert(e->is != false);

            if (e->is == false) {
                return -10;
            }

            Vert *v1 = e->tgt();
            Vert *v2 = e->src();
            assert(v1->is != false);
            assert(v2->is != false);

            // Iterator to be used.
            Edge *iter = nullptr;

            // Is this edge on boundary?
            if (e->isBoundary()) {
                return -1;
            }

            // Is the two verts on the boundary?
            if (v1->isBoundary()) {
                return -2;
            }

            if (v2->isBoundary()) {
                return -3;
            }

            // Check the length of this edge.
            if (e->len > maxLength) {
                return -4;
            }

            // Check topological conditions.
            std::vector<size_t> v1Neighbors;
            std::vector<size_t> v2Neighbors;

            // Add v1 neighbors into the set.
            iter = v1->edge;
            do {
                v1Neighbors.push_back(iter->vert->id);
                iter = iter->pair->next;
            } while (iter != v1->edge);
            std::sort(v1Neighbors.begin(), v1Neighbors.end());

            // Add v2 neighbors into the set.
            iter = v2->edge;
            do {
                v2Neighbors.push_back(iter->vert->id);
                iter = iter->pair->next;
            } while (iter != v2->edge);
            std::sort(v2Neighbors.begin(), v2Neighbors.end());

            // Find the intersect of these two sets.
            std::vector<size_t> intersection;
            std::set_intersection(v1Neighbors.begin(), v1Neighbors.end(),
                v2Neighbors.begin(), v2Neighbors.end(),
                std::back_inserter(intersection));

            // Check the size of the intersection.
            if (intersection.size() != 2) {
                return -5;
            }

            // Find the union of their neighbors.
            std::vector<size_t> neighborsUnion;
            std::set_union(v1Neighbors.begin(), v1Neighbors.end(),
                v2Neighbors.begin(), v2Neighbors.end(),
                std::back_inserter(neighborsUnion));

            // If the size of the union is 4, this is a tetrahedron. Don't collapse.
            if (neighborsUnion.size() <= 4) {
                return -6;
            }

            return 0;

        }

        /* Fix the link of edge of a vertex. In case it's not an circle. */
        inline void fixVertEdge(Vert *v) {
            Edge *edge = v->edge;
            while (edge->next->next->pair != nullptr) {
                edge = edge->next->next->pair;
                if (edge == v->edge)
                    break;
            }
            v->edge = edge;
        }

        /* Compute the Q for a vert. */
        void vertQ(Vert *v) {
            v->Q = glm::mat4(0.0f);
            Edge *edge = v->edge;

            while (edge != nullptr) {
                v->Q += edge->face->Q;
                if (edge->pair == nullptr) {
                    return;
                }
                edge = edge->pair->next;
                if (edge == v->edge) {
                    return;
                }
            }
            return;
        }

    };

};

#endif
