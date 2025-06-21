#pragma once

#include <string>
#include "struttura.hpp"
#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <map>
#include <vector>

// controlla se il punto (x,y,z) è già presente nella mesh
// se non lo è lo aggiunge e restituisce il suo id
namespace PolygonalLibrary {
unsigned int GetorCreatePoint(PolyhedralMesh& mesh, double& x, double& y,double& z, unsigned int l);
unsigned int GetorCreateEdge(PolyhedralMesh& mesh,unsigned int& x,unsigned int& y, unsigned int l);
// import da CSV con nome file esplicito
bool ImportCell0Ds(PolyhedralMesh& mesh, const std::string& filename);
bool ImportCell1Ds(PolyhedralMesh& mesh, const std::string& filename);
bool ImportCell2Ds(PolyhedralMesh& mesh, const std::string& filename);
bool Centralize(PolyhedralMesh& mesh);
bool Normalize(PolyhedralMesh& mesh);
void CreateTriFace(PolyhedralMesh& mesh,const int& p,const int& q, const int& b, const int& c);
void Export_Polyhedron(PolyhedralMesh& P);
// import di comodo che accetta il basename (es. "cube")
// e legge "<basename>_vertices.csv", "<basename>_edges.csv", "<basename>_faces.csv"
bool ImportMesh(PolyhedralMesh& mesh, const std::string& basename);
bool Triangulate(PolyhedralMesh& mesh,const int& p,const int& q, const int& b, const int& c);
void CalculateFaceCentroids(const PolyhedralMesh& originalMesh, PolyhedralMesh& dualMesh);
void CreateDualEdges(const PolyhedralMesh& originalMesh, PolyhedralMesh& dualMesh);
void CreateDualFaces(const PolyhedralMesh& originalMesh, PolyhedralMesh& dualMesh);
vector<unsigned int> OrderDualFaceVertices(const vector<unsigned int>& faceVertices, const PolyhedralMesh& dualMesh);
PolyhedralMesh CalculateDual(PolyhedralMesh& mesh, const int& p, const int& q, const int& b, const int& c);
double Distance(PolyhedralMesh& mesh, unsigned int id1, unsigned int id2);
unordered_map<unsigned int, unordered_set<unsigned int>> buildAdjacencyList(const vector<vector<unsigned int>>& Cell2DsVertices);
list<unsigned int> bfs_shortest_path(
    const unordered_map<unsigned int,unordered_set<unsigned int>>& adjacency,
    int start,int end,int n,
    map<unsigned int,list<unsigned int>>& Cell0DsMarker,map<unsigned int,list<unsigned int>>& Cell1DsMarker,
    unsigned int NumCell1Ds,MatrixXi Cell1DsExtrema);
bool inputdati(int &p, int &q, int &b, int &c, int &start, int &end, bool &search);
} // namespace PolygonalLibrary


