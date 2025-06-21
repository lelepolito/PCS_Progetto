#pragma once

#include <vector>
#include <map>
#include <list>
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

namespace PolygonalLibrary {

struct PolyhedralMesh {
    // CELLE 0D
    unsigned int NumCell0Ds = 0;
    vector<unsigned int> Cell0DsId;
    MatrixXd Cell0DsCoordinates;                  // 3 x NumCell0Ds
    map<unsigned int, list<unsigned int>> Cell0DsMarker;

    // CELLE 1D
    unsigned int NumCell1Ds = 0;
    vector<unsigned int> Cell1DsId;
    MatrixXi Cell1DsExtrema;                      // 2 x NumCell1Ds
    map<unsigned int, list<unsigned int>> Cell1DsMarker;

    // CELLE 2D
    unsigned int NumCell2Ds = 0;
    vector<unsigned int> Cell2DsId;
    vector<vector<unsigned int>> Cell2DsVertices;
    vector<vector<unsigned int>> Cell2DsEdges;

};

} // namespace PolygonalLibrary