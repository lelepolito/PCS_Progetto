#pragma once

#include <string>
#include "struttura.hpp"


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
void ProjectDualToSphere(PolyhedralMesh& dualMesh);
void ExportDualFiles(const PolyhedralMesh& dualMesh);
bool CalculateAndExportDual(PolyhedralMesh& mesh, const int& p, const int& q, const int& b, const int& c);
double Distance(PolyhedralMesh& mesh, unsigned int id1, unsigned int id2);


} // namespace PolygonalLibrary