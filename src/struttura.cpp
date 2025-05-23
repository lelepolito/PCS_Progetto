
#include <string>
#include "struttura.hpp"

namespace PolygonalLibrary {

bool ImportCell3Ds(PolyhedralMesh& mesh)
{
    mesh.NumCell3Ds = 1;
    mesh.Cell3DsId = { 0 };

    mesh.Cell3DsNumVertices = { mesh.NumCell0Ds };
    mesh.Cell3DsNumEdges    = { mesh.NumCell1Ds };
    mesh.Cell3DsNumFaces    = { mesh.NumCell2Ds };

    mesh.Cell3DsVertices.clear();
    mesh.Cell3DsEdges.clear();
    mesh.Cell3DsFaces.clear();

    mesh.Cell3DsVertices.push_back(mesh.Cell0DsId);
    mesh.Cell3DsEdges.push_back(mesh.Cell1DsId);
    mesh.Cell3DsFaces.push_back(mesh.Cell2DsId);

    return true;
}

} // namespace PolygonalLibrary
