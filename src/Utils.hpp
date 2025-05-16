#pragma once

#include <string>
#include "struttura.hpp"

namespace PolygonalLibrary {

// import da CSV con nome file esplicito
bool ImportCell0Ds(PolyhedralMesh& mesh, const std::string& filename);
bool ImportCell1Ds(PolyhedralMesh& mesh, const std::string& filename);
bool ImportCell2Ds(PolyhedralMesh& mesh, const std::string& filename);
bool Centralize(PolyhedralMesh& mesh);
bool Normalize(PolyhedralMesh& mesh);
// import di comodo che accetta il basename (es. "cube")
// e legge "<basename>_vertices.csv", "<basename>_edges.csv", "<basename>_faces.csv"
bool ImportMesh(PolyhedralMesh& mesh, const std::string& basename);
bool Triangulate(PolyhedralMesh& mesh, const int& b, const int& c);
} // namespace PolygonalLibrary