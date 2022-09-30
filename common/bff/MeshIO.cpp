/*  SPDX-License-Identifier: MIT

    The MIT License (MIT)

    Copyright (c) 2017 Rohan Sawhney and Keenan Crane

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
    Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
    PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
    OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MeshIO.h"
#include <unordered_map>
#include <queue>
#include <iomanip>

namespace bff {

void AdjacencyTable::construct(int n, const std::vector<int>& indices)
{
    data.resize(n);
    iMap.resize(n);
    size = 0;

    // build table
    for (int I = 0; I < (int)indices.size(); I += 3) {
        for (int J = 0; J < 3; J++) {
            int K = (J + 1) % 3;
            int i = indices[I + J];
            int j = indices[I + K];

            if (i > j) std::swap(i, j);
            data[i].emplace(j);
        }
    }

    // build iMap
    for (int i = 0; i < n; i++) {
        iMap[i] = size;
        size += data[i].size();
    }
}

int AdjacencyTable::getIndex(int i, int j) const
{
    if (i > j) std::swap(i, j);

    int k = 0;
    for (std::set<int>::iterator it = data[i].begin(); it != data[i].end(); it++) {
        if (*it == j) break;
        k++;
    }

    return iMap[i] + k;
}

int AdjacencyTable::getSize() const
{
    return size;
}

void MeshIO::separateComponents(const PolygonSoup& soup,
                                const std::vector<int>& isCuttableEdge,
                                std::vector<PolygonSoup>& soups,
                                std::vector<std::vector<int>>& isCuttableEdgeSoups,
                                std::vector<std::pair<int, int>>& modelToMeshMap,
                                std::vector<std::vector<int>>& meshToModelMap)
{
    // create an edge face adjacency map
    int nIndices = (int)soup.indices.size();
    int nVertices = (int)soup.positions.size();
    std::vector<std::vector<int>> adjacentFaces(soup.table.getSize());
    for (int I = 0; I < nIndices; I += 3) {
        for (int J = 0; J < 3; J++) {
            int K = (J + 1) % 3;
            int i = soup.indices[I + J];
            int j = soup.indices[I + K];

            int eIndex = soup.table.getIndex(i, j);
            adjacentFaces[eIndex].emplace_back(I);

            // check for non-manifold edges
            if (adjacentFaces[eIndex].size() > 2) {
                soups.emplace_back(soup);
                isCuttableEdgeSoups.emplace_back(isCuttableEdge);
                return;
            }
        }
    }

    // separate soup into components
    int components = 0;
    std::vector<bool> seenFace(nIndices, false);
    std::vector<int> faceComponent(nIndices);
    for (int I = 0; I < nIndices; I += 3) {
        // continue if face has already been seen
        if (seenFace[I]) continue;

        // collect all faces in a single component and mark them as seen
        seenFace[I] = true;
        faceComponent[I] = components;
        std::queue<int> q;
        q.push(I);

        while (!q.empty()) {
            int f = q.front();
            q.pop();

            // loop over all edges in the face
            for (int J = 0; J < 3; J++) {
                int K = (J + 1) % 3;
                int i = soup.indices[f + J];
                int j = soup.indices[f + K];

                int eIndex = soup.table.getIndex(i, j);

                // check if the edge is not on the boundary
                const std::vector<int>& faces = adjacentFaces[eIndex];
                if (faces.size() == 2) {
                    int g = f == faces[0] ? faces[1] : faces[0];

                    if (!seenFace[g]) {
                        seenFace[g] = true;
                        faceComponent[g] = components;
                        q.push(g);
                    }
                }
            }
        }

        components++;
    }

    meshToModelMap.resize(components);
    if (components == 1) {
        soups.emplace_back(soup);
        isCuttableEdgeSoups.emplace_back(isCuttableEdge);

        for (int i = 0; i < nVertices; i++) {
            meshToModelMap[0].emplace_back(i);
            modelToMeshMap.emplace_back(std::make_pair(0, i));
        }

    } else {
        // create soups
        soups.resize(components);
        isCuttableEdgeSoups.resize(components);
        std::vector<std::unordered_map<int, int>> seenVertex(components);
        modelToMeshMap.resize(nVertices);

        for (int I = 0; I < nIndices; I += 3) {
            int c = faceComponent[I];

            for (int J = 0; J < 3; J++) {
                int i = soup.indices[I + J];

                // insert vertex if it hasn't been seen
                if (seenVertex[c].find(i) == seenVertex[c].end()) {
                    int index = (int)soups[c].positions.size();
                    seenVertex[c][i] = index;
                    soups[c].positions.emplace_back(soup.positions[i]);
                    meshToModelMap[c].emplace_back(i);
                    modelToMeshMap[i] = std::make_pair(c, index);
                }

                // add index
                soups[c].indices.emplace_back(seenVertex[c][i]);
            }
        }

        // construct tables
        for (int c = 0; c < components; c++) {
            soups[c].table.construct(soups[c].positions.size(), soups[c].indices);
            isCuttableEdgeSoups[c].resize(soups[c].table.getSize(), 1);
        }

        // mark cuttable edges for each soup
        for (int I = 0; I < nIndices; I += 3) {
            int c = faceComponent[I];

            for (int J = 0; J < 3; J++) {
                int K = (J + 1) % 3;
                int i = soup.indices[I + J];
                int j = soup.indices[I + K];

                int eIndex = soup.table.getIndex(i, j);

                // add edge if uncuttable
                if (!isCuttableEdge[eIndex]) {
                    int ii = seenVertex[c][i];
                    int jj = seenVertex[c][j];

                    int eIndexSoup = soups[c].table.getIndex(ii, jj);
                    isCuttableEdgeSoups[c][eIndexSoup] = 0;
                }
            }
        }
    }
}

void MeshIO::preallocateElements(const PolygonSoup& soup, Mesh& mesh)
{
    // clear arrays
    mesh.vertices.clear();
    mesh.edges.clear();
    mesh.faces.clear();
    mesh.corners.clear();
    mesh.halfEdges.clear();
    mesh.boundaries.clear();

    // reserve space (reserving extra for hole filling and generators)
    int nVertices = (int)soup.positions.size();
    int nEdges = soup.table.getSize();
    int nFaces = (int)soup.indices.size()/3;
    int nCorners = 3*nFaces;
    int nHalfedges = 2*nEdges;

    mesh.vertices.reserve(2*nVertices);
    mesh.edges.reserve(2*nEdges);
    mesh.faces.reserve(2*nFaces);
    mesh.corners.reserve(2*nCorners);
    mesh.halfEdges.reserve(2*nHalfedges);
}

bool MeshIO::hasIsolatedVertices(const Mesh& mesh)
{
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        if (v->isIsolated()) {
            return true;
        }
    }

    return false;
}

bool MeshIO::hasNonManifoldVertices(const Mesh& mesh)
{
    VertexData<int> adjacentFaces(mesh, 0);
    for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        HalfEdgeCIter h = f->halfEdge();
        do {
            adjacentFaces[h->vertex()]++;

            h = h->next();
        } while (h != f->halfEdge());
    }

    for (BoundaryCIter b = mesh.boundaries.begin(); b != mesh.boundaries.end(); b++) {
        HalfEdgeCIter h = b->halfEdge();
        do {
            adjacentFaces[h->vertex()]++;

            h = h->next();
        } while (h != b->halfEdge());
    }

    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        if (adjacentFaces[v] != v->degree()) {
            return true;
        }
    }

    return false;
}

bool MeshIO::buildMesh(const PolygonSoup& soup,
                       const std::vector<int>& isCuttableEdge,
                       Mesh& mesh, std::string& error)
{
    // preallocate elements
    preallocateElements(soup, mesh);

    // create and insert vertices
    int nVertices = (int)soup.positions.size();
    std::vector<VertexIter> indexToVertex(nVertices);
    for (int i = 0; i < nVertices; i++) {
        VertexIter v = mesh.vertices.emplace(mesh.vertices.end(), Vertex(&mesh));
        v->position = soup.positions[i];
        v->index = (int)mesh.vertices.size() - 1;
        indexToVertex[i] = v;
    }

    // create and insert halfedges, edges and "real" faces
    int tableSize = soup.table.getSize();
    std::vector<int> edgeCount(tableSize, 0);
    std::vector<HalfEdgeIter> existingHalfEdges(tableSize);
    std::vector<int> hasFlipEdge(2*tableSize, 0);

    for (int I = 0; I < (int)soup.indices.size(); I += 3) {
        // create new face
        FaceIter f = mesh.faces.emplace(mesh.faces.end(), Face(&mesh));
        f->index = (int)mesh.faces.size() - 1;

        // create a halfedge for each edge of the newly created face
        std::vector<HalfEdgeIter> halfEdges(3);
        for (int J = 0; J < 3; J++) {
            halfEdges[J] = mesh.halfEdges.emplace(mesh.halfEdges.end(), HalfEdge(&mesh));
            halfEdges[J]->index = (int)mesh.halfEdges.size() - 1;
        }

        // initialize the newly created halfedges
        for (int J = 0; J < 3; J++) {
            // current halfedge goes from vertex i to vertex j
            int K = (J + 1) % 3;
            int i = soup.indices[I + J];
            int j = soup.indices[I + K];

            // set the current halfedge's attributes
            HalfEdgeIter h = halfEdges[J];
            h->setNext(halfEdges[K]);
            h->setPrev(halfEdges[(J + 3 - 1) % 3]);
            h->onBoundary = false;
            hasFlipEdge[h->index] = 0;

            // point the new halfedge and vertex i to each other
            VertexIter v = indexToVertex[i];
            h->setVertex(v);
            v->setHalfEdge(h);

            // point the new halfedge and face to each other
            h->setFace(f);
            f->setHalfEdge(h);

            int eIndex = soup.table.getIndex(i, j);
            if (edgeCount[eIndex] > 0) {
                // if a halfedge between vertex i and j has been created in the past,
                // then it is the flip halfedge of the current halfedge
                HalfEdgeIter flip = existingHalfEdges[eIndex];
                h->setFlip(flip);
                flip->setFlip(h);
                h->setEdge(flip->edge());

                hasFlipEdge[h->index] = 1;
                hasFlipEdge[flip->index] = 1;
                edgeCount[eIndex]++;

            } else {
                // create an edge and set its halfedge
                EdgeIter e = mesh.edges.emplace(mesh.edges.end(), Edge(&mesh));
                e->index = (int)mesh.edges.size() - 1;
                h->setEdge(e);
                e->setHalfEdge(h);
                e->isCuttable = isCuttableEdge[eIndex];

                // record the newly created edge and halfedge from vertex i to j
                existingHalfEdges[eIndex] = h;
                edgeCount[eIndex] = 1;
            }

            // check for non-manifold edges
            if (edgeCount[eIndex] > 2) {
                mesh.status = Mesh::ErrorCode::nonManifoldEdges;
                error = "Mesh has non-manifold edges.";
                return false;
            }
        }
    }

    // create and insert boundary halfedges and "imaginary" faces for boundary cycles
    // also create and insert corners
    for (HalfEdgeIter h = mesh.halfEdges.begin(); h != mesh.halfEdges.end(); h++) {
        // if a halfedge has no flip halfedge, create a new face and
        // link it the corresponding boundary cycle
        if (!hasFlipEdge[h->index]) {
            // create new face
            FaceIter f = mesh.boundaries.emplace(mesh.boundaries.end(), Face(&mesh));
            f->index = (int)mesh.boundaries.size() - 1;

            // walk along boundary cycle
            std::vector<HalfEdgeIter> boundaryCycle;
            boundaryCycle.reserve(2*mesh.edges.size() - mesh.halfEdges.size());
            HalfEdgeIter he = h;
            do {
                // create a new halfedge
                HalfEdgeIter bH = mesh.halfEdges.emplace(mesh.halfEdges.end(), HalfEdge(&mesh));
                bH->index = (int)mesh.halfEdges.size() - 1;
                boundaryCycle.emplace_back(bH);

                // grab the next halfedge along the boundary that does not have a
                // flip halfedge
                HalfEdgeIter nextHe = he->next();
                while (hasFlipEdge[nextHe->index]) {
                    nextHe = nextHe->flip()->next();
                }

                // set the current halfedge's attributes
                bH->setVertex(nextHe->vertex());
                bH->setEdge(he->edge());
                bH->onBoundary = true;

                // point the new halfedge and face to each other
                bH->setFace(f);
                f->setHalfEdge(bH);

                // point the new halfedge and he to each other
                bH->setFlip(he);
                he->setFlip(bH);

                // continue walk
                he = nextHe;
            } while (he != h);

            // link the cycle of boundary halfedges together
            int n = (int)boundaryCycle.size();
            for (int i = 0; i < n; i++) {
                boundaryCycle[i]->setNext(boundaryCycle[(i + n - 1) % n]); // boundary halfedges are linked in clockwise order
                boundaryCycle[i]->setPrev(boundaryCycle[(i + 1) % n]);
                hasFlipEdge[boundaryCycle[i]->index] = 1;
                hasFlipEdge[boundaryCycle[i]->flip()->index] = 1;
            }
        }

        // point the newly created corner and its halfedge to each other
        if (!h->onBoundary) {
            CornerIter c = mesh.corners.emplace(mesh.corners.end(), Corner(&mesh));
            c->index = (int)mesh.corners.size() - 1;
            c->setHalfEdge(h);
            h->setCorner(c);
        }
    }

    // check if mesh has isolated vertices
    if (hasIsolatedVertices(mesh)) {
        mesh.status = Mesh::ErrorCode::isolatedVertices;
        error = "Mesh has isolated vertices.";
        return false;
    }

    // check if mesh has non-manifold vertices
    if (hasNonManifoldVertices(mesh)) {
        mesh.status = Mesh::ErrorCode::nonManifoldVertices;
        error = "Mesh has non manifold vertices.";
        return false;
    }

    return true;
}

bool MeshIO::read(PolygonSoup& soup, Model& model, std::string& error)
{
    std::string line;
    int nVertices = 0;
    bool seenFace = false;
    std::set<std::pair<int, int>> uncuttableEdges;

    // construct table
    soup.table.construct(soup.positions.size(), soup.indices);
    std::vector<int> isCuttableEdge(soup.table.getSize(), 1);
    for (std::set<std::pair<int, int>>::iterator it = uncuttableEdges.begin();
                                                 it != uncuttableEdges.end();
                                                 it++) {
        int eIndex = soup.table.getIndex(it->first, it->second);
        isCuttableEdge[eIndex] = 0;
    }

    // separate model into components
    std::vector<PolygonSoup> soups;
    std::vector<std::vector<int>> isCuttableEdgeSoups;
    separateComponents(soup, isCuttableEdge, soups, isCuttableEdgeSoups,
                       model.modelToMeshMap, model.meshToModelMap);

    // build halfedge meshes
    model.meshes.resize(soups.size());
    for (int i = 0; i < (int)soups.size(); i++) {
        if (!buildMesh(soups[i], isCuttableEdgeSoups[i], model[i], error)) {
            return false;
        }
    }

    return true;
}

void MeshIO::normalize(Model& model)
{
    // compute center of mass
    Vector cm;
    int nVertices = 0;
    for (int i = 0; i < model.size(); i++) {
        for (VertexCIter v = model[i].vertices.begin(); v != model[i].vertices.end(); v++) {
            cm += v->position;
            nVertices++;
        }
    }
    cm /= nVertices;

    // translate to origin and determine radius
    double radius = 1e-8;
    for (int i = 0; i < model.size(); i++) {
        for (VertexIter v = model[i].vertices.begin(); v != model[i].vertices.end(); v++) {
            v->position -= cm;
            radius = std::max(radius, v->position.norm());
        }
    }

    // rescale to unit radius
    for (int i = 0; i < model.size(); i++) {
        for (VertexIter v = model[i].vertices.begin(); v != model[i].vertices.end(); v++) {
            v->position /= radius;
        }

        model[i].radius = radius;
        model[i].cm = cm;
    }
}

} // namespace bff
