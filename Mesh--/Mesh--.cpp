#include "Mesh--.h"
#include "Matrix.h"

namespace mmm {

    Mesh::Mesh(std::istream &in) {
        char c;
        while (in.get(c)) {
            if (c == '#') {
                char buf[1024];
                in.getline(buf, sizeof(buf));
                continue;
            }
            else if (c == '\n') {

            }
            else if (c == 'v') {
                double x;
                double y;
                double z;
                in >> x >> y >> z;

                vert.emplace_back(x, y, z);
            }
            else if (c == 'f') {
                int id[3];
                in >> id[0] >> id[1] >> id[2];

                if (face.size() != vert.size()) {
                    face.resize(vert.size());
                    removed.resize(vert.size(), false);
                    edge = new CrossLink((int)vert.size());
                }

                num_faces++;

                id[0]--; id[1]--; id[2]--;

                /**
                 * Store the opposite edge in the to represent the face.
                 * v, face[v].first, face[v].v2 is counter-clockwise.
                 */

                face[id[0]].emplace(id[1], id[2]);
                face[id[1]].emplace(id[2], id[0]);
                face[id[2]].emplace(id[0], id[1]);

                std::sort(id, id + 3);
                assert(0 <= id[0] && id[0] < id[1] && id[1] < id[2] && id[2] < vert.size());
                edge->insert(id[0], id[1]);
                edge->insert(id[1], id[2]);
                edge->insert(id[0], id[2]);

            }
            else {
                std::cerr << "Invalid Command: " << c << std::endl;
                continue;
            }
        }
        old_num_edges = edge->size();
        old_num_faces = face.size();
        old_num_verts = vert.size();
    }

    Mesh::~Mesh() {
        vert.clear();
        removed.clear();
        face.clear();
    }

    void Mesh::dumpObj(std::ostream &out) const {
        size_t vertexN = vert.size();
        std::vector<int> vertexID(vertexN, 0);
        int vertexReal = 0;

        for (int i = 0; i < vertexN; i++) {
            if (removed[i]) continue;
            vertexID[i] = ++vertexReal;
            out << "v " << vert[i][0] << " " << vert[i][1] << " " << vert[i][2] << std::endl;
        }

        for (int i = 0; i < vertexN; i++) {
            if (removed[i]) continue;
            for (const auto &f : face[i]) {
                assert(!removed[f.v1] && !removed[f.v2]);
                assert(vertexID[f.v1] && vertexID[f.v2] && vertexID[i]);
                if (i < f.v1 && i < f.v2) {
                    out << "f " << vertexID[i] << ' ' << vertexID[f.v1] << ' ' << vertexID[f.v2] << std::endl;
                }
            }
        }
    }

    /* Get the split position of the edge. */
    std::pair<Vector, double> Mesh::getPosition(const Edge &e) {

        /* Get the Q matrix for all the faces. */
        Matrix q(0.0);
        for (const auto &f : face[e.v1]) {
            auto n = crossProduct(vert[f.v1] - vert[e.v1], vert[f.v2] - vert[e.v1]);
            n = n / norm(n);
            n.push_back(-innerProduct(vert[e.v1], n));
            outerProductAcc(n, n, q);
        }
        for (const auto &f : face[e.v2]) {
            auto n = crossProduct(vert[f.v1] - vert[e.v2], vert[f.v2] - vert[e.v2]);
            n = n / norm(n);
            n.push_back(-innerProduct(vert[e.v2], n));
            outerProductAcc(n, n, q);
        }

        Vector v = (vert[e.v1] + vert[e.v2]) / 2;

        // Change this to vec4 and calculate the cost.
        v.push_back(1);
        double cost = innerProduct(innerProduct(v, q), v);
        assert(cost > -EPS);
        v.pop_back();
        return make_pair(v, cost);
    }


    /* Select an edge to collapse. */
    std::pair<Edge, Vector> Mesh::selectEdge(double threshold) {
        Edge idx(-1, -1);
        Vector pos;
        std::pair<double, Edge> tmp;
        while (!heap.empty()) {
            tmp = heap.top();
            heap.pop();

            // This edge is already deleted.
            if (!edge->find(tmp.second.v1, tmp.second.v2)) continue;

            // The verts of the edge has been deleted.
            if (removed[tmp.second.v1] || removed[tmp.second.v2]) continue;

            // The edge is too long.
            if (edgeLen(tmp.second) > threshold) continue;

            // Again we get the split position.
            auto act = getPosition(tmp.second);

            // Compare the new cost with the original one.
            if (fabs(act.second + tmp.first) > EPS) continue;

            // This is indeed a valid edge.
            idx = tmp.second;
            pos = act.first;
            break;
        }
        return std::make_pair(idx, pos);
    }

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
    bool Mesh::faceReverse(const Edge &e, const Vector &v1, const Vector &v2) {
        const auto &x = vert[e.v1];
        const auto &y = vert[e.v2];
        return innerProduct(crossProduct(x - v1, y - v1), crossProduct(x - v2, y - v2)) < 0.0;
    }

    /* Add an edge to the heap. */
    void Mesh::addToHeap(const Edge &e, double threshold) {
        if (edgeLen(e) > threshold) return;

        // Get the split position.
        auto pos = getPosition(e);

        heap.emplace(-pos.second, e);
    }

    /* Update the neighbor edge and add them to the heap. */
    void Mesh::updateNeighborEdge(int v, double threshold) {
        std::set<int> neighbor;
        for (const auto &f : face[v]) {
            neighbor.insert(f.v1);
            neighbor.insert(f.v2);
        }
        for (auto x : neighbor) {
            addToHeap(Edge(min(x, v), max(x, v)), threshold);
        }
    }

    /**
     * Remove an edge from the mesh.
     * @param e: edge to be removed.
     * @param v: new position for v1.
     * @param threshold: maximum length for a removable edge.
     * @return void.
     */
    void Mesh::removeEdge(const Edge &e, const Vector &v, double threshold) {

        // First check if any face of v1 will be reversed after this removal.
        std::vector<Edge> toRev;
        for (const auto &f : face[e.v1]) {

            // If this is the face we will remove, simply ignore it.
            if (f.v1 == e.v2 || f.v2 == e.v2) continue;

            bool reverse = faceReverse(f, vert[e.v1], v);
            if (!reverse) continue;

            // The face will be reversed after we replacing v1 with v.
            // Fix the orientation of the faces at face[f.v2] and face[f.v1].
            toRev.push_back(f);
            assert(face[f.v2].find(Edge(e.v1, f.v1)) != face[f.v2].end());
            face[f.v2].erase(Edge(e.v1, f.v1));
            face[f.v2].emplace(f.v1, e.v1);

            assert(face[f.v1].find(Edge(f.v2, e.v1)) != face[f.v1].end());
            face[f.v1].erase(Edge(f.v2, e.v1));
            face[f.v1].emplace(e.v1, f.v2);
        }

        for (const auto &f : toRev) {
            face[e.v1].erase(f);
            face[e.v1].emplace(f.v2, f.v1);
        }

        /* Process the faces of v2. 
         *          v2
         *         /  \
         *        /    \
         *      f.v1 - f.v2
         */
        for (const auto &f : face[e.v2]) {
            
            /* Check if the face will be reverse. */
            auto reverse = faceReverse(f, vert[e.v2], v);
            assert(face[f.v2].find(Edge(e.v2, f.v1)) != face[f.v2].end());
            assert(face[f.v1].find(Edge(f.v2, e.v2)) != face[f.v1].end());
            face[f.v2].erase(Edge(e.v2, f.v1));
            face[f.v1].erase(Edge(f.v2, e.v2));
            if (f.v1 != e.v1 && f.v2 != e.v1) {
                // This isn't the face we will collapse.
                if (reverse) {
                    face[f.v2].emplace(f.v1, e.v1);
                    face[f.v1].emplace(e.v1, f.v2);
                    face[e.v1].emplace(f.v2, f.v1);
                }
                else {
                    face[f.v2].emplace(e.v1, f.v1);
                    face[f.v1].emplace(f.v2, e.v1);
                    face[e.v1].insert(f);
                }
            }
            else {
                num_faces--;
            }

            // Update the edge.
            if (edge->find(min(e.v2, f.v1), max(e.v2, f.v1)))
                edge->erase(min(e.v2, f.v1), max(e.v2, f.v1));
            if (edge->find(min(e.v2, f.v2), max(e.v2, f.v2)))
                edge->erase(min(e.v2, f.v2), max(e.v2, f.v2));
            if (f.v1 != e.v1 && f.v2 != e.v1) {
                edge->insert(min(e.v1, f.v1), max(e.v1, f.v1));
                edge->insert(min(e.v1, f.v2), max(e.v1, f.v2));
            }
        }

        edge->erase(e.v1, e.v2);
        vert[e.v1] = v;
        vert[e.v2].clear();
        removed[e.v2] = true;
        face[e.v2].clear();

        std::set<int> neighbor;     /* All the neighboring vert. */
        for (const auto &f : face[e.v1]) {
            neighbor.insert(f.v1);
            neighbor.insert(f.v2);
        }
        for (auto nb : neighbor) {
            updateNeighborEdge(nb, threshold);
        }
    }

    /* Push all the valid edges into the heap. */
    void Mesh::buildHeap(double threshold) {

        // First clear the heap.
        while (!heap.empty()) heap.pop();

        for (int i = 0; i < edge->size(); ++i) {
            CrossLink::Node *node = edge->edgeX[i];
            while (node != nullptr) {
                addToHeap(Edge(node->x, node->y), threshold);
                node = node->dwn;
            }
        }
    }

    void Mesh::simplify(size_t target, double threshold) {

        // Build the heap.
        buildHeap(threshold);
        while (num_faces > target) {
            //  Select an edge to collapse.
            auto e = selectEdge(threshold);
            if (e.first.v1 != -1)
                removeEdge(e.first, e.second, threshold);
            else {
                // Failed getting an edge to deleted.
                // Double the threshold.
                threshold *= 2.0;
                std::cout << "Not enough edges, double the threshold.\n";
                buildHeap(threshold);
            }
        }
    }
};