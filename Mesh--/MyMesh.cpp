#include <algorithm>
#include <iomanip>
#include "MyMesh.h"

#define WIDTH 6

mm::Mesh::Mesh(std::istream &in, double ratio, double maxDistance)
    : ratio(ratio), maxDistance(maxDistance) {

    num_verts = 0;
    num_faces = 0;
    cut_verts = 0;
    cut_faces = 0;

    edgeX = nullptr;
    edgeY = nullptr;

    qVert = nullptr;
    qFace = nullptr;

    vfTable = nullptr;

    char c;
    while (in.get(c)) {
        if (c == '\n') {
            continue;
        }
        else if (c == 'v') {
            double x;
            double y;
            double z;
            in >> x >> y >> z;

            num_verts++;
            verts.emplace_back(x, y, z);

        }
        else if (c == 'f') {
            int id1;
            int id2;
            int id3;
            in >> id1 >> id2 >> id3;

            num_faces++;
            faces.emplace_back(id1 - 1, id2 - 1, id3 - 1);

        }
        else {
            std::cerr << "Invalid Command: " << c << std::endl;
            break;
        }
    }

    for (const auto &face : faces) {
        if (face.v1 < 0 || face.v1 >= num_verts) {
            std::cerr << "Illegal idx for vert. " << face.v1 << std::endl;
        }
        if (face.v2 < 0 || face.v2 >= num_verts) {
            std::cerr << "Illegal idx for vert. " << face.v2 << std::endl;
        }
        if (face.v3 < 0 || face.v3 >= num_verts) {
            std::cerr << "Illegal idx for vert. " << face.v3 << std::endl;
        }
    }

    /* Precompute everything. */
    precompute();
}

mm::Mesh::~Mesh() {
    // Release the cross link.
    if (edgeX != nullptr) {
        for (int i = 0; i < num_verts; ++i) {
            Edge *cur = edgeX[i];
            while (cur != nullptr) {
                Edge *dwn = cur->dwn;
                delete cur;
                cur = dwn;
            }
        }
        delete[] edgeX;
        delete[] edgeY;
    }

    // Release the vfTable.
    if (vfTable != nullptr) {
        for (int i = 0; i < num_verts; ++i) {
            vfTable[i].clear();
        }
        delete[] vfTable;
    }

    // Release all the other buffer.
    verts.clear();
    faces.clear();
    heap.clear();
}

void mm::Mesh::dumpObj(std::ostream &out) const {

    int *aux = new int[num_verts];
    aux[0] = 0;
    for (int i = 0; i < verts.size() - 1; ++i) {
        aux[i + 1] = verts[i].isDelted ? aux[i] + 1 : aux[i];
    }

    for (int i = 0; i < verts.size(); ++i) {
        if (!verts[i].isDelted) {
            out << "v " << 
                std::fixed << std::setprecision(5) << std::setw(WIDTH) << verts[i].v.x << ' ' <<
                std::fixed << std::setprecision(5) << std::setw(WIDTH) << verts[i].v.y << ' ' <<
                std::fixed << std::setprecision(5) << std::setw(WIDTH) << verts[i].v.z << std::endl;
        }
    }

    for (int i = 0; i < num_faces; ++i) {
        int v1 = faces[i].v1;
        int v2 = faces[i].v2;
        int v3 = faces[i].v3;
        if (v1 == v2 || v2 == v3 || v1 == v3) {
            continue;
        }

        double len1 = glm::distance(verts[v1].v, verts[v2].v);
        double len2 = glm::distance(verts[v1].v, verts[v3].v);
        double len3 = glm::distance(verts[v2].v, verts[v3].v);
        
        double total_len = len1 + len2 + len3;
        double threshold = ave_len * 5;

        if (len1 > threshold ||
            len2 > threshold ||
            len3 > threshold) {
            std::cout << "Illegal faces." << std::endl;
        }

        int id1 = v1 - aux[v1] + 1;
        int id2 = v2 - aux[v2] + 1;
        int id3 = v3 - aux[v3] + 1;
        
        int total = num_verts - aux[num_verts - 1];

        out << "f " << id1 << ' ' << id2 << ' ' << id3 << std::endl;
    }

    delete[] aux;
}

void mm::Mesh::precompute() {
    if (num_faces == 0 || num_verts == 0) {
        std::cerr << "The mesh is empty." << std::endl;
        return;
    }

    edgeX = new Edge*[num_verts];
    edgeY = new Edge*[num_verts];

    vfTable = new std::vector<int>[num_verts];

    qVert = new glm::mat4[num_verts];
    qFace = new glm::mat4[num_faces];

    for (int i = 0; i < num_verts; ++i) {
        vfTable[i].clear();
        edgeX[i] = nullptr;
        edgeY[i] = nullptr;
        qVert[i] = glm::mat4(0.0f);
    }

    ave_len = 0.0;
    size_t num_edges = 0;
    for (int i = 0; i < num_faces; ++i) {

        int id1 = faces[i].v1;
        int id2 = faces[i].v2;
        int id3 = faces[i].v3;

        faces[i].computeP(verts[id1], verts[id2], verts[id3]);
        qFace[i] = glm::mat4(
            faces[i].p * faces[i].p.x,
            faces[i].p * faces[i].p.y,
            faces[i].p * faces[i].p.z,
            faces[i].p * faces[i].p.w
            );

        qVert[id1] += qFace[i];
        qVert[id2] += qFace[i];
        qVert[id3] += qFace[i];

        Edge *e1 = new Edge(id1, id2);
        Edge *e2 = new Edge(id1, id3);
        Edge *e3 = new Edge(id2, id3);

        e1->length = glm::distance(verts[id1].v, verts[id2].v);
        e2->length = glm::distance(verts[id1].v, verts[id3].v);
        e3->length = glm::distance(verts[id2].v, verts[id3].v);
        ave_len += e1->length;
        ave_len += e2->length;
        ave_len += e3->length;
        num_edges += 3;


        vfTable[id1].push_back(i);
        vfTable[id2].push_back(i);
        vfTable[id3].push_back(i);

        // Push the edge into the cross link.
        // Edge.v1 for x axis, Edge.v2 for y axis.
        if (insertEdge(e1)) {
            delete e1;
        }
        if (insertEdge(e2)) {
            delete e2;
        }
        if (insertEdge(e3)) {
            delete e3;
        }

    }

    ave_len /= (double)num_edges;
}

void mm::Mesh::restoreHeap(int id) {

    if (id < 0 || id >= heap.size()) {
        return;
    }

    if (id > 0 && heap[(id - 1) / 2].cost > heap[id].cost) {
        while (id > 0) {
            int parent = (id - 1) / 2;
            if (heap[parent].cost > heap[id].cost) {
                heap[id].edge->heapIdx = parent;
                heap[parent].edge->heapIdx = id;
                swap(heap[id], heap[parent]);
                id = parent;
            }
            else {
                break;
            }
        }
    }
    else if (id * 2 + 1 < heap.size()) {
        int lhs = id * 2 + 1;
        int rhs = lhs + 1;
        while (lhs < heap.size()) {
            if (rhs < heap.size()) {
                if (heap[lhs].cost < heap[id].cost && heap[lhs].cost < heap[rhs].cost) {
                    heap[id].edge->heapIdx = lhs;
                    heap[lhs].edge->heapIdx = id;
                    swap(heap[id], heap[lhs]);
                    id = lhs;
                    lhs = id * 2 + 1;
                    rhs = lhs + 1;
                }
                else if (heap[rhs].cost < heap[id].cost && heap[rhs].cost < heap[lhs].cost) {
                    heap[id].edge->heapIdx = rhs;
                    heap[rhs].edge->heapIdx = id;
                    swap(heap[id], heap[rhs]);
                    id = rhs;
                    lhs = id * 2 + 1;
                    rhs = lhs + 1;
                }
                else {
                    break;
                }
            }
            else {
                if (heap[lhs].cost < heap[id].cost) {
                    heap[id].edge->heapIdx = lhs;
                    heap[lhs].edge->heapIdx = id;
                    swap(heap[id], heap[lhs]);
                }
                break;
            }
        }
    }
}

void mm::Mesh::buildHeap() {
    for (int i = 0; i < num_verts; ++i) {
        Edge *cur = edgeX[i];
        while (cur != nullptr) {
            if (cur->length < maxDistance) {
                int v1 = cur->v1;
                int v2 = cur->v2;
                double cost = cur->compute(verts[v1], verts[v2], qVert[v1], qVert[v2]);
                heap.emplace_back(cost, cur);
                cur->heapIdx = (int)heap.size() - 1;
                restoreHeap((int)heap.size() - 1);
            }
            cur = cur->dwn;
        }
    }
}

void mm::Mesh::simplify() {
    double remain = (double)num_verts * ratio;
    if (remain < 10.0f) {
        std::cerr << "Over simplified! " << std::endl;
        return;
    }
    while (num_verts - cut_verts >= remain) {
        if (heap.size() == 0) {
            maxDistance *= 2;
            buildHeap();
        }
        else {
            int result = collapse();
            if (result >= 0) {
                cut_verts++;
                cut_faces += result;
            }
        }
    }
}

int mm::Mesh::collapse() {
    int id1 = heap[0].edge->v1;
    int id2 = heap[0].edge->v2;

    // Is this edge valid?
    if (id1 == id2) {
        popHeap(0);
        return -1;
    }
    else {
        int num_deleted = 0;        // # Faces deleted.
        verts[id1].v = heap[0].edge->split;
        verts[id2].isDelted = true;
        popHeap(0);

        for (int i = 0; i < vfTable[id1].size(); ++i) {
            int fId = vfTable[id1][i];
            int vId1 = faces[fId].v1;
            int vId2 = faces[fId].v2;
            int vId3 = faces[fId].v3;
            if (vId1 != vId2 && vId1 != vId3 && vId2 != vId3) {
                qVert[vId1] -= qFace[fId];
                qVert[vId2] -= qFace[fId];
                qVert[vId3] -= qFace[fId];
            }
        }
        for (int i = 0; i < vfTable[id2].size(); ++i) {
            int fId = vfTable[id2][i];
            int vId1 = faces[fId].v1;
            int vId2 = faces[fId].v2;
            int vId3 = faces[fId].v3;
            if (vId1 != vId2 && vId1 != vId3 && vId2 != vId3 &&
                vId1 != id1 && vId2 != id1 && vId3 != id1) {
                qVert[vId1] -= qFace[fId];
                qVert[vId2] -= qFace[fId];
                qVert[vId3] -= qFace[fId];
            }
        }

        // Merge the faces of v2 to v1;
        for (int i = 0; i < vfTable[id2].size(); ++i) {
            int fId = vfTable[id2][i];
            if (faces[fId].v1 == id2) {
                faces[fId].v1 = id1;
            }
            if (faces[fId].v2 == id2) {
                faces[fId].v2 = id1;
            }
            if (faces[fId].v3 == id2) {
                faces[fId].v3 = id1;
            }

            int vId1 = faces[fId].v1;
            int vId2 = faces[fId].v2;
            int vId3 = faces[fId].v3;
            if (vId1 == vId2 || vId1 == vId3 || vId2 == vId3) {
                num_deleted++;
                continue;
            }
            for (int j = 0; ; ++j) {
                if (j == vfTable[id1].size()) {
                    vfTable[id1].push_back(fId);
                    break;
                }
                int _fId = vfTable[id1][j];
                if (compareFaces(faces[fId], faces[_fId])) {
                    break;
                }
            }
        }
        vfTable[id2].clear();

        // Update the new q matrix.
        for (int i = 0; i < vfTable[id1].size(); ++i) {
            int fId = vfTable[id1][i];
            int vId1 = faces[fId].v1;
            int vId2 = faces[fId].v2;
            int vId3 = faces[fId].v3;
            if (vId1 != vId2 && vId1 != vId3 && vId2 != vId3) {
                faces[fId].computeP(verts[vId1], verts[vId2], verts[vId3]);
                qFace[fId] = glm::mat4(
                    faces[fId].p * faces[fId].p.x,
                    faces[fId].p * faces[fId].p.y,
                    faces[fId].p * faces[fId].p.z,
                    faces[fId].p * faces[fId].p.w);
                qVert[vId1] += qFace[fId];
                qVert[vId2] += qFace[fId];
                qVert[vId3] += qFace[fId];
            }
        }

        // Update the new cost for the edge.
        // First, find all the edges containing v2 and get them out of the cross link.
        // For the (v2, y).
        Edge *pEdgeX2 = edgeX[id2];
        while (pEdgeX2 != nullptr) {
            if (pEdgeX2 == edgeY[pEdgeX2->v2]) {
                edgeY[pEdgeX2->v2] = pEdgeX2->rhs;
            }
            if (pEdgeX2->rhs != nullptr) {
                pEdgeX2->rhs->lhs = pEdgeX2->lhs;
            }
            if (pEdgeX2->lhs != nullptr) {
                pEdgeX2->lhs->rhs = pEdgeX2->rhs;
            }
            pEdgeX2->v1 = id1;
            pEdgeX2 = pEdgeX2->dwn;
        }

        // For the (x, id2).
        Edge *pEdgeY2 = edgeY[id2];
        while (pEdgeY2 != nullptr) {
            if (pEdgeY2 == edgeX[pEdgeY2->v1]) {
                edgeX[pEdgeY2->v1] = pEdgeY2->dwn;
            }
            if (pEdgeY2->ups != nullptr) {
                pEdgeY2->ups->dwn = pEdgeY2->dwn;
            }
            if (pEdgeY2->dwn != nullptr) {
                pEdgeY2->dwn->ups = pEdgeY2->ups;
            }
            pEdgeY2->v2 = id1;
            if (pEdgeY2->v2 < pEdgeY2->v1) {
                int temp = pEdgeY2->v1;
                pEdgeY2->v1 = pEdgeY2->v2;
                pEdgeY2->v2 = temp;
            }
            pEdgeY2 = pEdgeY2->rhs;
        }

        pEdgeX2 = edgeX[id2];
        pEdgeY2 = edgeY[id2];
        edgeX[id2] = nullptr;
        edgeY[id2] = nullptr;

        // New edges with (id1, y).
        while (pEdgeX2 != nullptr) {
            Edge *dwn = pEdgeX2->dwn;
            pEdgeX2->lhs = pEdgeX2->rhs = pEdgeX2->ups = pEdgeX2->dwn = nullptr;
            if (pEdgeX2->v1 == pEdgeX2->v2) {
                // This is the edge we collapsed.
                int heapIdx = pEdgeX2->heapIdx;
                if (heapIdx >= 0) {
                    popHeap(heapIdx);
                }
                delete pEdgeX2;
            }
            else {
                // They are new edges to v1. Insert them back into the cross link.
                bool duplicated = insertEdge(pEdgeX2);
                if (duplicated) {
                    // If duplicated, remove it from the heap and delete it.
                    int heapId = pEdgeX2->heapIdx;
                    if (heapId >= 0) {
                        popHeap(heapId);
                    }

                    // TODO: Figure out what the hell is this for?
                    if (pEdgeX2 == edgeX[pEdgeX2->v1]) {
                        edgeX[pEdgeX2->v1] = dwn;
                    }

                    delete pEdgeX2;
                }

            }

            // Move down to next one.
            pEdgeX2 = dwn;
        }

        // New edges with (x, id1).
        while (pEdgeY2 != nullptr) {
            Edge *rhs = pEdgeY2->rhs;
            pEdgeY2->lhs = pEdgeY2->rhs = pEdgeY2->ups = pEdgeY2->dwn = nullptr;
            if (pEdgeY2->v1 == pEdgeY2->v2) {
                // This is the edge we collapsed.
                int heapId = pEdgeY2->heapIdx;
                if (heapId >= 0) {
                    popHeap(heapId);
                }
                delete pEdgeY2;
            }
            else {
                // Insert it into the cross link.
                bool duplicated = insertEdge(pEdgeY2);
                if (duplicated) {
                    int heapId = pEdgeY2->heapIdx;
                    if (heapId >= 0) {
                        popHeap(heapId);
                    }

                    // TODO: Figure out what the hell is this for?
                    if (pEdgeY2 == edgeX[pEdgeY2->v1]) {
                        edgeX[pEdgeY2->v1] = pEdgeY2->dwn;
                    }

                    delete pEdgeY2;
                }
            }
            pEdgeY2 = rhs;
        }

        // Update the cost for edges with (id1, y).
        Edge *pEdgeX1 = edgeX[id1];
        while (pEdgeX1 != nullptr) {
            int vId1 = pEdgeX1->v1;
            int vId2 = pEdgeX1->v2;
            double cost = pEdgeX1->compute(verts[vId1], verts[vId2], qVert[vId1], qVert[vId2]);
            pEdgeX1->length = glm::distance(verts[vId1].v, verts[vId2].v);

            int heapId = pEdgeX1->heapIdx;
            if (heapId < 0 && pEdgeX1->length < maxDistance) {
                // This is a new edge. Push it into the heap.
                heap.emplace_back(cost, pEdgeX1);
                pEdgeX1->heapIdx = (int)heap.size() - 1;
                restoreHeap((int)heap.size() - 1);
            }
            else if (heapId >= 0) {
                // This is an old edge in the heap.
                // Update the cost.
                heap[heapId].cost = cost;
                if (pEdgeX1->length > maxDistance) {
                    // The adjusted edge is too long.
                    // Remove it from the heap.
                    popHeap(heapId);
                }
                else {
                    restoreHeap(heapId);
                }
            }

            pEdgeX1 = pEdgeX1->dwn;
        }

        // Updates the cost for edges with (x, id1).
        Edge *pEdgeY1 = edgeY[id1];
        while (pEdgeY1 != nullptr) {

            int vId1 = pEdgeY1->v1;
            int vId2 = pEdgeY1->v2;
            double cost = pEdgeY1->compute(verts[vId1], verts[vId2], qVert[vId1], qVert[vId2]);
            pEdgeY1->length = glm::distance(verts[vId1].v, verts[vId2].v);

            int heapId = pEdgeY1->heapIdx;
            if (heapId < 0 && pEdgeY1->length < maxDistance) {
                // This is a new edge. Push it into the heap.
                heap.emplace_back(cost, pEdgeY1);
                pEdgeY1->heapIdx = (int)heap.size() - 1;
                restoreHeap((int)heap.size() - 1);
            }
            else if (heapId >= 0) {
                // This is an old edge in the heap.
                // Update the cost.
                heap[heapId].cost = cost;
                if (pEdgeY1->length > maxDistance) {
                    // The adjusted edge is too long.
                    // Remove it from the heap.
                    popHeap(heapId);
                }
                else {
                    restoreHeap(heapId);
                }
            }

            pEdgeY1 = pEdgeY1->rhs;
        }

        return num_deleted;

    }
}

bool mm::Mesh::compareFaces(const Face &f1, const Face &f2) const {

    return (f1.v1 == f2.v1 && f1.v2 == f2.v2 && f1.v3 == f2.v3) ||
        (f1.v1 == f2.v1 && f1.v2 == f2.v3 && f1.v3 == f2.v2) ||
        (f1.v1 == f2.v2 && f1.v2 == f2.v1 && f1.v3 == f2.v3) ||
        (f1.v1 == f2.v2 && f1.v2 == f2.v3 && f1.v3 == f2.v1) ||
        (f1.v1 == f2.v3 && f1.v2 == f2.v1 && f1.v3 == f2.v2) ||
        (f1.v1 == f2.v3 && f1.v2 == f2.v2 && f1.v3 == f2.v1);
}

void mm::Mesh::popHeap(int id) {
    if (id < 0 || id >= heap.size()) {
        return;
    }
    else if (id == heap.size() - 1) {
        heap[id].edge->heapIdx = -1;
        heap.pop_back();
    }
    else {
        heap[heap.size() - 1].edge->heapIdx = id;
        heap[id].edge->heapIdx = -1;
        swap(heap[id], heap[heap.size() - 1]);
        heap.pop_back();
        restoreHeap(id);
    }
}

bool mm::Mesh::insertEdge(Edge *e) {

    int x = e->v1;
    int y = e->v2;

    Edge *px = edgeX[x];
    Edge *py = edgeY[y];

    bool duplicated = false;
    if (px == nullptr) {
        edgeX[x] = e;
    }
    else {
        while (px != nullptr) {
            if (px->v2 > y) {
                e->dwn = px;
                edgeX[x]->ups = e;
                edgeX[x] = e;
                break;
            }
            else if (px->v2 == y) {
                duplicated = true;
                break;
            }
            else {
                if (px->dwn == nullptr) {
                    px->dwn = e;
                    e->ups = px;
                    break;
                }
                else if (px->dwn->v2 > y) {
                    e->dwn = px->dwn;
                    e->ups = px;
                    px->dwn->ups = e;
                    px->dwn = e;
                    break;
                }
                else {
                    px = px->dwn;
                }
            }
        }
    }

    if (duplicated) {
        return duplicated;
    }
    else if (py == nullptr) {
        edgeY[y] = e;
    }
    else {
        while (py != nullptr) {
            if (py->v1 > x) {
                e->rhs = edgeY[y];
                edgeY[y]->lhs = e;
                edgeY[y] = e;
                break;
            }
            else if (py->v1 == x) {
                break;
            }
            else {
                if (py->rhs == nullptr) {
                    e->lhs = py;
                    py->rhs = e;
                    break;
                }
                else if (py->rhs->v1 > x) {
                    e->rhs = py->rhs;
                    e->lhs = py;
                    py->rhs->lhs = e;
                    py->rhs = e;
                    break;
                }
                else {
                    py = py->rhs;
                }
            }
        }
    }
    return duplicated;
}