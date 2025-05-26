#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include "UCDUtilities.hpp"
#include "Utils.hpp"
#include "struttura.hpp"
#include <iomanip>
using namespace std;

unordered_map<unsigned int, unordered_set<unsigned int>> adjacency;
//function lista di adiacenza
void buildAdjacencyList(const vector<vector<unsigned int>>& Cell2DsVertices) {
    for (const auto& face : Cell2DsVertices) {
        size_t numVertices = face.size();

        for (size_t i = 0; i < numVertices; ++i) {
            unsigned int u = face[i];
            unsigned int v = face[(i + 1) % numVertices]; //serve per chiudere il

            // Aggiungi arco bidirezionale
            adjacency[u].insert(v);
            adjacency[v].insert(u);
        }
    }
}

//function per cammino minmo tra due vertici
std::vector<int> bfs_shortest_path(
    const std::unordered_map<unsigned int, std::unordered_set<unsigned int>>& adjacency,
    int start,
    int end,
    int n
    ) {std::vector<int> predecessor(n, -1);
        std::vector<bool> visited(n, false);
    
        std::queue<int> q;
        q.push(start);
        visited[start] = true;
    
        while (!q.empty()) {
            int u = q.front();
            q.pop();
    
            for (int v : adjacency.at(u)) {
                if (!visited[v]) {
                    visited[v] = true;
                    predecessor[v] = u;
                    q.push(v);
    
                    if (v == end) {  // Fermati appena trovi il nodo finale
                        q = std::queue<int>(); // svuota la coda forzatamente
                        break;
                    }
                }
            }
        }
    
        // Se end non è stato raggiunto
        if (!visited[end]) {
            return {}; // Cammino vuoto = nessun cammino trovato
        }
    
        // Ricostruisci il cammino minimo da start a end
        std::vector<int> path;     
        for (int at = end; at != -1; at = predecessor[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        return path; 
    }








// legge 4 interi p, q, b, c  devo avere p, q ∈ [3,5]
bool quadinput(int &p, int &q, int &b, int &c, int&start, int&end) {
    cout << "Inserisci sei interi p, q, b, c, start, end: ";
    if (!(cin >> p >> q >> b >> c >> start >> end)) {
        cerr << "Errore di lettura: inserisci sei numeri interi." << endl;
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

    int p, q, b, c, start, end;
    if (!quadinput(p, q, b, c, start, end)) {
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

    // 4. Costruisci lista di adiacenza (popola adjacency)
    buildAdjacencyList(mesh.Cell2DsVertices);

    cout << "Lista di adiacenza:\n";
    for (const auto& pair : adjacency) {
        unsigned int u = pair.first;
        const auto& neighbors = pair.second;

        cout << "Vertice " << u << " è adiacente a: ";
        for (unsigned int v : neighbors) {
            cout << v << " ";
    }
    cout << "\n";
}


// 5. Ora puoi controllare se start e end esistono nel grafo (adjacency)
    if (adjacency.find(start) == adjacency.end() || adjacency.find(end) == adjacency.end()) {
        cerr << "Errore: uno dei vertici non esiste nel grafo." << endl;
        return EXIT_FAILURE;
}

// 6. Calcolo cammino minimo
    unsigned int n = mesh.Cell3DsNumVertices[0];
    std::vector<int> path = bfs_shortest_path(adjacency, start, end, n);

// 7. Stampa risultato
    if (path.empty()) {
        cout << "Nessun cammino esiste tra " << start << " e " << end << ".\n";
}   else {
        cout << "\nCammino minimo tra " << start << " e " << end << ": ";
        for (int v : path) {
            cout << v << " ";
    }
        cout << "\nLunghezza (in spigoli): " << path.size() - 1 << "\n";
}



    if (PolygonalLibrary::Centralize(mesh)) {
        cout << "Mesh centrato." << endl;
    } else {
        cout << "Mesh non centrato." << endl;
    }
    
    if (!PolygonalLibrary::Triangulate(mesh,p,q, b, c)) {
        cerr << "Triangulation fallita per " << basename << endl;
        return EXIT_FAILURE;
    } else {
        cout << "Triangulation riuscita." << endl;
    }

    if (PolygonalLibrary::Normalize(mesh)) {
        cout << "Mesh normalizzato." << endl;
    } else {
        cout << "Mesh non normalizzato." << endl;
    }

    Gedim::UCDUtilities utilities;
    Export_Polyhedron(mesh);
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