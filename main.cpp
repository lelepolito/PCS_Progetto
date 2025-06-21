#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include<queue>
#include "UCDUtilities.hpp"
#include "Utils.hpp"
#include "struttura.hpp"
#include <iomanip>
using namespace std;
using namespace PolygonalLibrary;

// legge 4 interi p, q, b, c  devo avere p, q ∈ [3,5]
bool input(int &p, int &q, int &b, int &c, int &start, int &end, bool &search) {
    const int DEFAULT_START = 0;
    const int DEFAULT_END   = 0;
    cout << "Inserisci p, q, b, c [start end] (start ed end sono opzionali): ";
    string line;
    // Legge tutta la riga di input
    if (!getline(cin, line) || line.empty()) {
        // Se resta un '\n' nel buffer, rilancia una getline
        if (!getline(cin, line)) {
            cerr << "Errore di lettura della riga di input." << endl;
            return false;
        }
    }

    istringstream iss(line);
    // Legge i 4 valori obbligatori
    if (!(iss >> p >> q >> b >> c)) {
        cerr << "Errore: devi inserire almeno quattro numeri interi (p, q, b, c)." << endl;
        return false;
    }
    // Prova a leggere start e end; se fallisce, usa i default
    if (!(iss >> start >> end)) {
        start = DEFAULT_START;
        end   = DEFAULT_END;
        search = false; // indica che non sono stati inseriti start ed end
    }

    // Controlli su p e q
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


int main() {
    //   p*10 + q --> basename dei file CSV
    const map<int, string> schlafiMap = {
        {33, "db/tetrahedron"},
        {43, "db/cube"},
        {34, "db/octahedron"},
        {53, "db/dodecahedron"},
        {35, "db/icosahedron"}
    };

    int p, q, b, c, start, end;
    bool search = true;
    if (!inputdati(p, q, b, c, start, end, search)) {
        return EXIT_FAILURE;
    }

    // Calcolo del codice per schlafiMap
    int code;
    if (q == 3) {
        // Caso duale: devo triangolare {3,p} per ottenere il duale {p,3}
        code = 3 * 10 + p;
    } else {
        // Caso standard
        code = p * 10 + q;
    }
    
    auto it = schlafiMap.find(code);
    if (it == schlafiMap.end()) {
        cerr << "La coppia {" << p << "," << q << "} non corrisponde a un solido platonico." << endl;
        return EXIT_FAILURE;
    }
    const string basename = it->second;  // nome del solido (cube, octahedron, ...)
    cout << "Hai scelto: " << basename << endl;

    PolyhedralMesh mesh;
    if (!ImportMesh(mesh, basename)) {
        cerr << "ImportMesh fallito per " << basename << endl;
        return EXIT_FAILURE;
    }
    if (Centralize(mesh)) {
        cout << "Mesh centrato." << endl;
    } else {
        cout << "Mesh non centrato." << endl;
    }
    
    // Triangolazione: se q==3 triangolo {3,p}, altrimenti {p,q}
    if (q == 3) {
        cout << "Triangulating {3," << p << "} to calculate dual {" << p << ",3}" << endl;
        if (!Triangulate(mesh, 3, p, b, c)) {
            cerr << "Triangulation fallita per {3," << p << "}" << endl;
            return EXIT_FAILURE;
        }
    } else {
        cout << "Triangulating standard polyhedron {" << p << "," << q << "}" << endl;
        if (!Triangulate(mesh, p, q, b, c)) {
            cerr << "Triangulation fallita per " << basename << endl;
            return EXIT_FAILURE;
        }
    }


    // Export dei file .txt 

    
    // GESTIONE DEL DUALE quando q == 3
    if (q == 3) {
        cout << "Calculating dual of {3," << p << "} to obtain {" << p << ",3}" << endl;
        
        mesh = CalculateDual(mesh, p, q, b, c);


    } else {
        cout << "Standard polyhedron construction (no dual calculation needed)." << endl;
    }
    cout << "Triangulation riuscita." << endl;

    if (Normalize(mesh)) {
        cout << "Mesh normalizzato." << endl;
    } else {
        cout << "Mesh non normalizzato." << endl;
    }

if (search == true) {
    // 4. Costruisci lista di adiacenza (popola adjacency)
    unordered_map<unsigned int, unordered_set<unsigned int>> adjacency = buildAdjacencyList(mesh.Cell2DsVertices);

    cout << "Lista di adiacenza:\n";
    for (const auto& [u, neighbors] : adjacency) {
        cout << "Vertice " << u << " è adiacente a: ";
        for (int v : neighbors) {
            cout << v << " ";
        }
        cout << endl;
    }

    // 5. Ora puoi controllare se start e end esistono nel grafo (adjacency)
    if (adjacency.find(start) == adjacency.end() || adjacency.find(end) == adjacency.end()) {
        cerr << "Errore: uno dei vertici non esiste nel grafo." << endl;
        return EXIT_FAILURE;
    }

    // 6. Calcolo cammino minimo

    unsigned int n = 0;
    for (const auto& [u, _] : adjacency) {
        if (u + 1 > n) n = u + 1;
    }

    bfs_shortest_path(adjacency, start, end, n, mesh.Cell0DsMarker, mesh.Cell1DsMarker,
                                          mesh.NumCell1Ds, mesh.Cell1DsExtrema);

        
    //ShortPath property
    vector<Gedim::UCDProperty<double>> points_properties;
    vector<Gedim::UCDProperty<double>> segmnents_properties;
    points_properties.reserve(1); //We have only one Property
    segmnents_properties.reserve(1);

    vector<double> prop_vert(mesh.NumCell0Ds, 0.0);
    vector<double> prop_edges(mesh.NumCell1Ds, 0.0);

    //Fill the struct points_properties
    Gedim::UCDProperty<double> pointP;
    pointP.Label = "ShortPath";
    pointP.NumComponents = 1;
    pointP.Data = prop_vert.data();
    points_properties.push_back(pointP);


    //Fill the struct segments_properties
   Gedim::UCDProperty<double> edgeP;
   edgeP.Label = "ShortPath";
   edgeP.NumComponents = 1;
   edgeP.Data = prop_edges.data();
   segmnents_properties.push_back(edgeP);   


    for (const auto& pair : mesh.Cell0DsMarker) {
        unsigned int marker = pair.first;
        const list<unsigned int>& vert_ids = pair.second;

        if (marker == 1) {
            for (unsigned int id : vert_ids) {
                if (id < prop_vert.size()) {
                    prop_vert[id] = 1.0; // imposta il valore per identificare i vertici marcati
                }
            }
        }
    }
    for (const auto& pair : mesh.Cell1DsMarker) {
        unsigned int marker = pair.first;
        const list<unsigned int>& edge_ids = pair.second;

        if (marker == 1) {
           for (unsigned int id : edge_ids) {
               if (id < prop_edges.size()) {
                   prop_edges[id] = 1.0; 
               }
           }
        }
    }
    cout << "Cammino minimo tra " << start << " e " << end << ": ";
    // Export dei file .inp per Paraview 
    Gedim::UCDUtilities utilities;
    utilities.ExportPoints("./Cell0Ds.inp",
                           mesh.Cell0DsCoordinates,
                             points_properties);

    utilities.ExportSegments("./Cell1Ds.inp",
                             mesh.Cell0DsCoordinates,
                             mesh.Cell1DsExtrema,
                             points_properties,
                             segmnents_properties);
    if (q != 3) {
        // Solo se non è il caso del duale {3,p}
        utilities.ExportPolygons("./Cell2Ds.inp",
                                mesh.Cell0DsCoordinates,
                                mesh.Cell2DsVertices);
    }
}
   
    if (search == false) {
        Gedim::UCDUtilities utilities;
        utilities.ExportPoints("./Cell0Ds.inp",
                                mesh.Cell0DsCoordinates);

        utilities.ExportSegments("./Cell1Ds.inp",
                                mesh.Cell0DsCoordinates,
                                mesh.Cell1DsExtrema);
        if (q != 3) {
            utilities.ExportPolygons("./Cell2Ds.inp",
                                    mesh.Cell0DsCoordinates,
                                    mesh.Cell2DsVertices);
        }
    }
    Export_Polyhedron(mesh);
    cout << "File generati: Cell0Ds.txt, Cell1Ds.txt, Cell2Ds.txt, Cell3Ds.txt" << endl;
    cout << "File Paraview: Cell0Ds.inp, Cell1Ds.inp, Cell2Ds.inp" << endl;
    
    return EXIT_SUCCESS;
}