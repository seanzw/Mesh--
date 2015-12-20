#include <vector>
#include <fstream>
#include <iomanip>
#include "HalfEdgeMesh--.h"

//#ifdef _DEBUG
//std::ostream &LOG = std::ofstream("log.txt");
//#else
//std::ostream nowhere(0);
//std::ostream &LOG = nowhere;
//#endif

std::ostream &LOG = std::ofstream("log.txt");

#define WIDTH 3
#define TIMES_RATIO 10.0f
namespace mmm {

    void HalfEdgeMesh::Edge::dump(std::ostream &out) {
        out << "Edge " << std::setw(WIDTH) << id;
        if (is) {
            Vert *from = next->next->vert;
            size_t pairId = pair != nullptr ? pair->id : 0;
            out << " From " << std::setw(WIDTH) << from->id << " to " << std::setw(WIDTH) << vert->id;
            out << " Face " << std::setw(WIDTH) << face->id;
            out << " Pair " << std::setw(WIDTH) << pairId;
            out << " Heap " << std::setw(WIDTH) << heapId;
        }
        out << std::endl;
    }

    void HalfEdgeMesh::Edge::computeE() {
        e = vert->pos - next->next->vert->pos;
        len = glm::length(e);
    }

    size_t HalfEdgeMesh::Vert::degree() const {
        Edge *e = edge;
        size_t degree = 0;
        do {
            degree++;
            if (e->pair == nullptr) {
                break;
            }
            else {
                e = e->pair->next;
            }
        } while (e != edge);
        return degree;
    }

    void HalfEdgeMesh::Vert::dump(std::ostream &out) {
        out << "Vert " << std::setw(WIDTH) << id << ' ' << is << std::endl;
        if (is) {
            Edge *e = this->edge;
            do {
                out << "    Edge " << std::setw(WIDTH) << e->id << " to " << std::setw(WIDTH) << e->vert->id << " ";
                e->dump(out);
                if (e->pair == nullptr) {
                    break;
                }
                else {
                    e = e->pair->next;
                }
            } while (e != this->edge);
        }
    }

    void HalfEdgeMesh::Face::dump(std::ostream &out) {
        out << "Face " << std::setw(WIDTH) << id << ' ' << is << std::endl;
        if (is) {
            Edge *e = this->edge;
            do {
                out << "   Edge " << std::setw(WIDTH) << e->id << " Vert " << std::setw(WIDTH) << e->vert->id << std::endl;
                e = e->next;
            } while (e != this->edge);
        }
    }

    void HalfEdgeMesh::MinHeap::dump(std::ostream &out) const {
        out << "Heap size: " << std::setw(WIDTH) << size << std::endl;
        for (size_t i = 0; i < size; ++i) {
            out << "    Heap ";
            buffer[i].edge->dump(out);
            out << "   "
                << " v1 degree " << std::setw(WIDTH) << buffer[i].edge->vert->degree()
                << " v2 degree " << std::setw(WIDTH) << buffer[i].edge->pair->vert->degree() << std::endl;
        }
    }

    bool HalfEdgeMesh::MinHeap::selfCheck(std::ostream &out, double maxLength) const {

        out << "Heap self-check " << std::endl;
        bool status = true;
        for (size_t i = 0; i < size; ++i) {
            if (HalfEdgeMesh::isEdgeNotCollapsable(buffer[i].edge, maxLength)) {
                status = false;
                out << HalfEdgeMesh::isEdgeNotCollapsable(buffer[i].edge, maxLength);
                out << "    Heap ";
                buffer[i].edge->dump(out);
                out << "   "
                    << " v1 degree " << std::setw(WIDTH) << buffer[i].edge->vert->degree()
                    << " v2 degree " << std::setw(WIDTH) << buffer[i].edge->pair->vert->degree() << std::endl;
            }
            if (buffer[i].edge->heapId != i ||
                buffer[i].edge->pair->heapId != i) {
                status = false;
                out << "    Heap Wrong heapId ";
                buffer[i].edge->dump(out);
                out << "    Heap Wrong heapId for pair ";
                buffer[i].edge->pair->dump(out);
            }
            if (i > 0) {
                size_t parent = (i - 1) / 2;
                if (buffer[i].cost < buffer[parent].cost) {
                    status = false;
                    out << "    child cost: " << buffer[i].cost << " parent cost: " << buffer[parent].cost << std::endl;
                }
            }

        }
        return status;
    }

    HalfEdgeMesh::HalfEdgeMesh(std::istream &in) {

        verts = nullptr;
        faces = nullptr;
        edges = nullptr;

        num_faces = 0;
        num_edges = 0;
        num_verts = 0;

        maxLength = 0.05;

        std::vector<Vert *> ids;

        std::map<std::tuple<int, int, int>, Face *> faceMap;

        char c;
        while (in.get(c)) {
            if (c == '#') {
                char buf[1024];
                in.getline(buf, sizeof(buf));
                continue;
            }
            else if (c == '\n') {
                continue;
            }
            else if (c == 'v') {
                double x;
                double y;
                double z;
                in >> x >> y >> z;

                Vert *newVert = new Vert(glm::vec3(x, y, z) * TIMES_RATIO);
                newVert->link = verts;
                verts = newVert;
                num_verts++;
                ids.push_back(newVert);

            }
            else if (c == 'f') {
                int id1;
                int id2;
                int id3;
                in >> id1 >> id2 >> id3;

                {
                    // Check if there is already this face.
                    int m1 = std::min(std::min(id1, id2), id3);
                    int m3 = std::max(std::max(id2, id2), id3);
                    int m2 = id1 + id2 + id3 - m1 - m3;
                    auto key = std::make_tuple(m1, m2, m3);
                    auto iter = faceMap.find(key);
                    if (iter != faceMap.end()) {
                        continue;
                    }
                    else {
                        faceMap[key] = nullptr;
                    }
                }


                Vert *v1 = ids[id1 - 1];
                Vert *v2 = ids[id2 - 1];
                Vert *v3 = ids[id3 - 1];

                Face *face = new Face();
                face->link = faces;
                faces = face;
                num_faces++;

                // Create the three edges.
                Edge *edge1 = make_edge(v1, v2, face);
                Edge *edge2 = make_edge(v2, v3, face);
                Edge *edge3 = make_edge(v3, v1, face);

                num_edges += 3;

                // Connect them together.
                edge1->next = edge2;
                edge2->next = edge3;
                edge3->next = edge1;

                // Set face to the edge.
                face->edge = edge1;

                // Calculate Q for this face.
                face->computeQ();
            }
            else {
                std::cerr << "Invalid Command: " << c << std::endl;
                break;
            }
        }

        // Connect the edges to its pair.
        Edge *edge = edges;
        while (edge != nullptr) {

            Edge *prev = edge;
            while (prev->next != edge) {
                prev = prev->next;
            }
            Vert *v1 = prev->vert;
            Vert *v2 = edge->vert;

            auto key = std::make_pair(v1, v2);
            edgeHashMap.emplace(key, edge);

            key = std::make_pair(v2, v1);
            auto iter = edgeHashMap.find(key);
            if (iter != edgeHashMap.end()) {
                Edge *pair = iter->second;
                edge->pair = pair;
                pair->pair = edge;
            }

            edge = edge->link;
        }

        /* Fix the problem for vert's edge. */
        Vert *vert = verts;
        while (vert != nullptr) {
            fixVertEdge(vert);
            vert = vert->link;
        }

        old_num_edges = num_edges;
        old_num_faces = num_faces;
        old_num_verts = num_verts;

#ifdef DEBUG
        dumpDebug();
#endif
    }

    HalfEdgeMesh::~HalfEdgeMesh() {

        while (verts != nullptr) {
            Vert *vert = verts;
            verts = verts->link;
            delete vert;
        }

        while (faces != nullptr) {
            Face *face = faces;
            faces = faces->link;
            delete face;
        }

        while (edges != nullptr) {
            Edge *edge = edges;
            edges = edges->link;
            delete edge;
        }
    }

    void HalfEdgeMesh::dumpObj(std::ostream &out) const {

        Vert *vert = verts;
        size_t id = 0;

        std::map<Vert *, size_t> idMap;

        while (vert != nullptr) {
            if (vert->is) {
                out << "v " << vert->pos.x / TIMES_RATIO
                    << ' ' << vert->pos.y / TIMES_RATIO
                    << ' ' << vert->pos.z / TIMES_RATIO << std::endl;
                idMap.insert(std::make_pair(vert, ++id));
            }
            vert = vert->link;
        }

        Face *face = faces;
        while (face != nullptr) {
            if (face->is) {
                Vert *v1 = face->edge->vert;
                Vert *v2 = face->edge->next->vert;
                Vert *v3 = face->edge->next->next->vert;
                if (v1 == v2 || v1 == v3 || v2 == v3) {
                    face = face->link;
                    continue;
                }
                Edge *edge = face->edge;
                out << "f";
                do {
                    auto iter = idMap.find(edge->vert);
                    if (iter == idMap.end()) {
                        std::cerr << "Missing Vert " << edge->vert->id << " Face " << face->id << std::endl;
                        return;
                    }
                    else {
                        out << ' ' << iter->second;
                        edge = edge->next;
                    }
                } while (edge != face->edge);
                out << std::endl;
            }
            face = face->link;
        }
    }

    void HalfEdgeMesh::dumpDebug() const {

        LOG << "Verts " << num_verts << std::endl;
        Vert *vert = verts;
        while (vert != nullptr) {
            vert->dump(LOG);
            vert = vert->link;
        }

        LOG << "Faces " << num_faces << std::endl;
        Face *face = faces;
        while (face != nullptr) {
            face->dump(LOG);
            face = face->link;
        }

        LOG << "Edges " << num_edges << std::endl;
        Edge *edge = edges;
        while (edge != nullptr) {
            edge->dump(LOG);
            edge = edge->link;
        }
    }

    void HalfEdgeMesh::buildHeap(MinHeap &heap, double maxLength) {
        // For all the edges.
        Edge *edge = edges;
        size_t debug = 0;
        while (edge != nullptr) {

            // Only process the edge when
            // 1. It's not in the heap yet.
            // 2. It's collapsable.
            if (edge->heapId == -1 && !isEdgeNotCollapsable(edge, maxLength)) {

                Vert *v1 = edge->vert;
                Vert *v2 = edge->pair->vert;

                // For now, choose the middle point
                glm::vec4 v((v1->pos + v2->pos) * 0.5f, 1.0f);

                // Compute the cost for this vert.
                double cost = dot(v, (v1->Q + v2->Q) * v);

                // Add this into the heap.
                heap.emplace_back(edge, glm::vec3(v), cost);

            }
            edge = edge->link;
        }
    }

    void HalfEdgeMesh::simplify(size_t remain_verts) {

        // Reserve the space for min-heap.
        MinHeap heap((num_edges >> 1) + 1);

        // Compute the Q for all verts.
        Vert *vert = verts;
        while (vert != nullptr) {
            vertQ(vert);
            vert = vert->link;
        }

        buildHeap(heap, maxLength);

        // Remove pairs from the heap.
        while (num_verts > remain_verts) {

            // Pop a new edge.
            if (heap.size == 0) {
                maxLength *= 2.0;
                if (maxLength >= 10.0f * TIMES_RATIO) {
                    // There is no more edge collapsable.
                    std::cout << "Warning: There is no more edge collapsable! ";
                    std::cout << "Current result will be saved. " << std::endl;
                    break;
                }
                else {
                    buildHeap(heap, maxLength);
                    continue;
                }
            }
            Pair pair = heap.pop(0);

            if (collapse(pair, heap) == 0) {
                num_faces -= 2;
                num_verts -= 1;
                num_edges -= 6;
            }

#ifdef _DEBUG
            //dumpDebug();
            //static int debug = 0;
            //debug++;
            //std::string debugObj("debug");
            //char buf[100];
            //_itoa_s(debug, buf, 10);
            //debugObj.append(buf);
            //debugObj.append(".obj");
            //std::ofstream debugOut(debugObj);
            //dumpObj(debugOut);
            //debugOut.close();
            //heap.dump(LOG);
            //if (!heap.selfCheck(std::cout, maxLength)) {
            //    std::cout << "Wrong!" << std::endl;
            //}
#endif

            
        }
    }

    int HalfEdgeMesh::collapse(const Pair &pair, MinHeap &heap) {

        if (isEdgeNotCollapsable(pair.edge, maxLength)) {
            return -1;
        }

        Edge *iter = nullptr;

        Edge *e = pair.edge;
        Edge *o = pair.edge->pair;
        Edge *en = e->next;
        Edge *ep = en->next;
        Edge *on = o->next;
        Edge *op = on->next;
        Edge *eno = en->pair;
        Edge *epo = ep->pair;
        Edge *opo = op->pair;
        Edge *ono = on->pair;

        Vert *v1 = pair.edge->tgt();
        Vert *v2 = pair.edge->src();
        Vert *vl = en->vert;
        Vert *vr = on->vert;

        /**
         * This is like:
         *                         v1
         *                      *  *  *          
         *                     //  ||  \\        
         *                    //   ||   \\       
         *                   //    ||    \\                           
         *               eno//     ||     \\opo  
         *                 //en    ||    op\\    
         *                //       ||       \\   
         *               //        ||        \\  
         *               *         ||          *
         *             vl         e||o         vr
         *              *          ||         * 
         *               \\        ||        // 
         *                \\       ||       //  
         *                 \\ep    ||    on//   
         *               epo\\     ||     //ono  
         *                   \\    ||    //     
         *                    \\   ||   //      
         *                     \\  ||  //       
         *                       *  *  *        
         *                         v2
         *
         *
         */

#ifdef _DEBUG
        if (pair.edge->id == 55095) {
            std::cout << "Debug" << std::endl;
        }
#endif
        // Check if the new position will inverse the triangle.
        auto checkInversion = [](Edge *e, const Vec3 &pos)->bool {
            Vec3 v1 = e->vert->pos;
            Vec3 v2 = e->next->vert->pos;
            Vec3 v3 = e->next->next->vert->pos;
            Vec3 n1 = glm::normalize(glm::cross(v2 - v1, v3 - v1));
            Vec3 n2 = glm::normalize(glm::cross(v2 - v1, pos - v1));
            return glm::dot(n1, n2) < 0.0;
        };

        // Check this for v2.
        iter = e;
        do {
            if (iter != e && iter != on) {
                if (checkInversion(iter, pair.v)) {
                    return -1;
                }
            }
            iter = iter->pair->next;
        } while (iter != e);

        // Check this for v1.
        iter = o;
        do {
            if (iter != o && iter != en) {
                if (checkInversion(iter, pair.v)) {
                    return -1;
                }
            }
            iter = iter->pair->next;
        } while (iter != o);

        /************************************************/
        /* Start the actual collapse.                   */

        // Switch off the second vert.
        v2->is = false;
        v2->edge = nullptr;

        // Set v1 to the new vert.
        v1->pos = pair.v;
        v1->Q += v2->Q;

        // Iterator.
        Edge *edge = nullptr;

        // For the faces affected by this collapse, remove the Q from their verts.
        iter = e;
        do {
            Vert *vert1 = iter->vert;
            Vert *vert2 = iter->next->vert;
            Vert *vert3 = iter->next->next->vert;
            vert1->Q -= iter->face->Q;
            vert2->Q -= iter->face->Q;
            vert3->Q -= iter->face->Q;
            iter = iter->pair->next;
        } while (iter != e);

        // Carefully we don't minus twice for the faces collapsed.
        iter = o;
        do {
            if (iter != o && iter != en) {
                Vert *vert1 = iter->vert;
                Vert *vert2 = iter->next->vert;
                Vert *vert3 = iter->next->next->vert;
                vert1->Q -= iter->face->Q;
                vert2->Q -= iter->face->Q;
                vert3->Q -= iter->face->Q;
            }
            iter = iter->pair->next;
        } while (iter != o);
        

        // Switch off the two faces.
        e->face->is = false;
        e->face->edge = nullptr;
        o->face->is = false;
        o->face->edge = nullptr;

        // Swich off the six edges.
        auto closeEdge = [&heap](Edge *e)->void {
            if (e->heapId >= 0) {
                heap.pop(e->heapId);
            }
            e->vert = nullptr;
            e->pair = nullptr;
            e->face = nullptr;
            e->next = nullptr;
            e->is = false;
            assert(e->heapId == -1);
        };

        closeEdge(e);
        closeEdge(en);
        closeEdge(ep);

        closeEdge(o);
        closeEdge(op);
        closeEdge(on);

        /* After this point no pair.edge should be used. */

        // Now glue the edges of v2 to v1.
        eno->pair = epo;
        epo->pair = eno;
        opo->pair = ono;
        ono->pair = opo;

        // Now fix the vert field of the edges of v2.
        iter = ono;
        while (iter != eno) {
            iter->vert = v1;
            iter = iter->next->pair;
        }

        // Now fix the edge field of v1, vl and vr.
        v1->edge = opo;
        vl->edge = eno;
        fixVertEdge(vl);
        vr->edge = ono;
        fixVertEdge(vr);

        v1->dump(LOG);
        LOG << std::endl;

        assert(v1->selfCheck() == 0);
        assert(v1->isBoundary() == false);

        // Cost for all edges.
        auto updateEdge = [&heap](Edge *edge, double maxLength)->void {
            Vert *vv1 = edge->vert;
            Vert *vv2 = edge->next->next->vert;
            glm::mat4 Q = vv1->Q + vv2->Q;
            glm::vec3 v = (vv1->pos + vv2->pos) * 0.5f;
            double cost = glm::dot(glm::vec4(v, 1.0f), Q * glm::vec4(v, 1.0f)) * edge->face->area() * edge->len;
            int heapId = edge->heapId;

            if (heapId == -1) {
                // It's not in the heap yet.
                // If it's collapsable, push it into the heap.
                if (!isEdgeNotCollapsable(edge, maxLength)) {
                    heap.emplace_back(edge, v, cost);
                }
                else {
                }
            }
            else {
                // It's already in the heap.
                // Check if it's collapsable and update the heap.
                if (isEdgeNotCollapsable(edge, maxLength)) {
                    heap.update(edge->heapId, v, cost);
                }
                else {
                    heap.pop(edge->heapId);
                }
            }
        };

        // Now update the E for edges.
        iter = opo;
        do {
            iter->computeE();
            iter->pair->computeE();
            iter = iter->pair->next;
        } while (iter != opo);

        // Q for faces.
        iter = opo;
        do {
            iter->face->computeQ();
            iter = iter->pair->next;
        } while (iter != opo);

        // Q for all verts.
        v1->Q = glm::mat4(0.0f);
        iter = opo;
        do {
            iter->vert->Q += iter->face->Q;
            iter->next->vert->Q += iter->face->Q;
            iter->next->next->vert->Q += iter->face->Q;
            iter = iter->pair->next;
        } while (iter != opo);

        iter = opo;
        do {
            updateEdge(iter, maxLength);
            updateEdge(iter->next, maxLength);
            updateEdge(iter->next->next, maxLength);

            iter = iter->pair->next;
        } while (iter != opo);

#ifdef _DEBUG
        if (!heap.selfCheck(std::cout, maxLength)) {
            std::cout << "Wrong!" << std::endl;
        }
#endif
        return 0;
    }
};