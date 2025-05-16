#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include "UCDUtilities.hpp"
#include "Utils.hpp"
#include "struttura.hpp"

using namespace std;

// legge 4 interi p, q, b, c  devo avere p, q ∈ [3,5]
bool quadinput(int &p, int &q, int &b, int &c) {
    cout << "Inserisci quattro interi p, q, b, c: ";
    if (!(cin >> p >> q >> b >> c)) {
        cerr << "Errore di lettura: inserisci quattro numeri interi." << endl;
        return false;
    }
    if (p < 3 || p > 5) {
        cerr << "Valore di p non valido (" << p << "): deve essere 3, 4 o 5." << endl;
        return false;
    }
    if (q < 3 || q > 5) {
        cerr << "Valore di q non valido (" << q << "): deve essere 3, 4 o 5." << endl;
        return false;
    }
    return true;
}

// dump(A;B) copie dei file CSV A per i file output.txt B 
bool dumpCSV(const string &infile, const string &outfile) {
    ifstream fin(infile);
    if (!fin.is_open()) {
        cerr << "Impossibile aprire " << infile << endl;
        return false;
    }
    ofstream fout(outfile);
    if (!fout.is_open()) {
        cerr << "Impossibile creare " << outfile << endl;
        return false;
    }
    fout << fin.rdbuf();  // copia il contenuto di infile in fout
    return true;
}

int main() {
    //   p*10 + q --> basename dei file CSV
    const map<int, string> schlafiMap = {
        {33, "db/tetrahedron"},
        {43, "db/cube"},
        {34, "db/octahedron"},
        {53, "db/dodecahedron"},
        {35, "db/icosahedron"}
    };

    int p, q, b, c;
    if (!quadinput(p, q, b, c)) {
        return EXIT_FAILURE;
    }

    int code = p * 10 + q;
    auto it = schlafiMap.find(code);
    if (it == schlafiMap.end()) {
        cerr << "La coppia {" << p << "," << q << "} non corrisponde a un solido platonico." << endl;
        return EXIT_FAILURE;
    }
    const string basename = it->second;  // nome del solido (cube, octahedron, ...)
    cout << "Hai scelto: " << basename << endl;

    PolygonalLibrary::PolyhedralMesh mesh;
    if (!PolygonalLibrary::ImportMesh(mesh, basename)) {
        cerr << "ImportMesh fallito per " << basename << endl;
        return EXIT_FAILURE;
    }
    if (!PolygonalLibrary::ImportCell3Ds(mesh)) {
        cerr << "ImportCell3Ds fallito per " << basename << endl;
        return EXIT_FAILURE;
    }
    if (PolygonalLibrary::Centralize(mesh)) {
        cout << "Mesh centrato." << endl;
    } else {
        cout << "Mesh non centrato." << endl;
    }
    if (PolygonalLibrary::Normalize(mesh)) {
        cout << "Mesh normalizzato." << endl;
    } else {
        cout << "Mesh non normalizzato." << endl;
    }
    // Dump cell0, cell1, cell2
    if (!dumpCSV(basename + "_vertices.csv", "cell0.txt")) return EXIT_FAILURE;
    if (!dumpCSV(basename + "_edges.csv",    "cell1.txt")) return EXIT_FAILURE;
    if (!dumpCSV(basename + "_faces.csv",    "cell2.txt")) return EXIT_FAILURE;

    // popoliamo il file relativo alle celle 3D
    ofstream f3("cell3.txt");
    if (!f3.is_open()) {
        cerr << "Impossibile creare cell3.txt" << endl;
        return EXIT_FAILURE;
    }
    f3 << "Polyhedron: " << basename << "\n";
    f3 << "NumCell3Ds: " << mesh.NumCell3Ds << "\n";
    f3 << "Cell3DsId: ";
    for (auto id : mesh.Cell3DsId) f3 << id << " ";
    f3 << "\nNumVertices: "  << mesh.Cell3DsNumVertices[0]
       << ", NumEdges: "  << mesh.Cell3DsNumEdges[0]
       << ", NumFaces: "  << mesh.Cell3DsNumFaces[0] << "\n";
    f3 << "Vertex IDs: ";
    for (auto vid : mesh.Cell3DsVertices[0]) f3 << vid << " ";
    f3 << "\nEdge IDs: ";
    for (auto eid : mesh.Cell3DsEdges[0]) f3 << eid << " ";
    f3 << "\nFace IDs: ";
    for (auto fid : mesh.Cell3DsFaces[0]) f3 << fid << " ";
    f3 << endl;
    Gedim::UCDUtilities utilities;

    utilities.ExportPoints("./Cell0Ds.inp",
                           mesh.Cell0DsCoordinates);

    utilities.ExportSegments("./Cell1Ds.inp",
                             mesh.Cell0DsCoordinates,
                             mesh.Cell1DsExtrema);
    utilities.ExportPolygons("./Cell2Ds.inp",
                              mesh.Cell0DsCoordinates,
                              mesh.Cell2DsVertices);
    cout << "File generati: cell0.txt, cell1.txt, cell2.txt, cell3.txt" << endl;
    return EXIT_SUCCESS;
}