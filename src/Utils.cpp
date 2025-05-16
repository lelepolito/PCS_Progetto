
#include "Utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
using namespace std;


namespace PolygonalLibrary {




bool Centralize(PolyhedralMesh& mesh)
{
    if (mesh.NumCell0Ds == 0) return false;


    double xsum;
    double ysum;
    double zsum;

    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
        xsum += mesh.Cell0DsCoordinates(0, i);
        ysum += mesh.Cell0DsCoordinates(1, i);
        zsum += mesh.Cell0DsCoordinates(2, i);
    }



    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
        mesh.Cell0DsCoordinates(0, i) -= (xsum / mesh.NumCell0Ds);
        mesh.Cell0DsCoordinates(1, i) -= (ysum / mesh.NumCell0Ds);
        mesh.Cell0DsCoordinates(2, i) -= (zsum / mesh.NumCell0Ds);
    }
    return true;
}
bool Normalize(PolyhedralMesh& mesh)
{
    mesh.Cell0DsCoordinates = mesh.Cell0DsCoordinates * 1e5;
    if (mesh.NumCell0Ds == 0) return false;
    double norm;
    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
        norm = sqrt(
            mesh.Cell0DsCoordinates(0, i) * mesh.Cell0DsCoordinates(0, i) +
            mesh.Cell0DsCoordinates(1, i) * mesh.Cell0DsCoordinates(1, i) +
            mesh.Cell0DsCoordinates(2, i) * mesh.Cell0DsCoordinates(2, i)
        );
        mesh.Cell0DsCoordinates(0, i) /= norm;
        mesh.Cell0DsCoordinates(1, i) /= norm;
        mesh.Cell0DsCoordinates(2, i) /= norm;
    }
    return true;
}
bool ImportCell0Ds(PolyhedralMesh& mesh, const string& filename)
{
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Cannot open vertex file: " << filename << endl;
        return false;
    }
    string line;
    getline(file, line); // salta header

    vector<unsigned int> ids;
    vector<int> markers;
    vector<double> xs, ys, zs;
    while (getline(file, line)) {
        stringstream ss(line);
        unsigned int id; int marker;
        double x, y, z;
        char sep;
        ss >> id >> sep >> marker >> sep >> x >> sep >> y >> sep >> z;
        ids.push_back(id);
        markers.push_back(marker);
        xs.push_back(x);
        ys.push_back(y);
        zs.push_back(z);
    }

    mesh.NumCell0Ds = ids.size();
    mesh.Cell0DsId = move(ids);
    mesh.Cell0DsCoordinates.resize(3, mesh.NumCell0Ds);
    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
        mesh.Cell0DsCoordinates(0, i) = xs[i];
        mesh.Cell0DsCoordinates(1, i) = ys[i];
        mesh.Cell0DsCoordinates(2, i) = zs[i];
        mesh.Cell0DsMarker[markers[i]].push_back(mesh.Cell0DsId[i]);
    }
    return true;
}

bool ImportCell1Ds(PolyhedralMesh& mesh, const string& filename)
{
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Cannot open edge file: " << filename << endl;
        return false;
    }
    string line;
    getline(file, line); // header

    vector<unsigned int> ids, origins, ends;
    vector<int> markers;
    while (getline(file, line)) {
        stringstream ss(line);
        unsigned int id, o, e; int marker;
        char sep;
        ss >> id >> sep >> marker >> sep >> o >> sep >> e;
        ids.push_back(id);
        markers.push_back(marker);
        origins.push_back(o);
        ends.push_back(e);
    }

    mesh.NumCell1Ds = ids.size();
    mesh.Cell1DsId = move(ids);
    mesh.Cell1DsExtrema.resize(2, mesh.NumCell1Ds);
    for (size_t i = 0; i < mesh.NumCell1Ds; ++i) {
        mesh.Cell1DsExtrema(0, i) = origins[i];
        mesh.Cell1DsExtrema(1, i) = ends[i];
        mesh.Cell1DsMarker[markers[i]].push_back(mesh.Cell1DsId[i]);
    }
    return true;
}

bool ImportCell2Ds(PolyhedralMesh& mesh, const string& filename)
{
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Cannot open face file: " << filename << endl;
        return false;
    }
    string line;
    getline(file, line); // header

    while (getline(file, line)) {
        stringstream ss(line);
        unsigned int id; int marker, numV, numE;
        char sep;
        ss >> id >> sep >> marker >> sep >> numV;
        vector<unsigned int> verts(numV);
        for (int i = 0; i < numV; ++i) ss >> sep >> verts[i];
        ss >> sep >> numE;
        vector<unsigned int> edges(numE);
        for (int i = 0; i < numE; ++i) ss >> sep >> edges[i];

        mesh.Cell2DsId.push_back(id);
        mesh.Cell2DsVertices.push_back(move(verts));
        mesh.Cell2DsEdges.push_back(move(edges));
        mesh.Cell2DsMarker[marker].push_back(id);
    }
    mesh.NumCell2Ds = mesh.Cell2DsId.size();
    return true;
}

bool ImportMesh(PolyhedralMesh& mesh, const string& basename)
{
    if (!ImportCell0Ds(mesh, basename + "_vertices.csv")) return false;
    if (!ImportCell1Ds(mesh, basename + "_edges.csv"))    return false;
    if (!ImportCell2Ds(mesh, basename + "_faces.csv"))    return false;
    return true;
}

} // namespace PolygonalLibrary