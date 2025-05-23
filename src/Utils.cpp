
#include "Utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <iomanip>

using namespace std;
using namespace Eigen;

namespace PolygonalLibrary 
{

    // calcola la distanza tra due punti 
double Distance(PolyhedralMesh& mesh, unsigned int id1, unsigned int id2)
{
    double x1 = mesh.Cell0DsCoordinates(0, id1);
    double y1 = mesh.Cell0DsCoordinates(1, id1);
    double z1 = mesh.Cell0DsCoordinates(2, id1);
    double x2 = mesh.Cell0DsCoordinates(0, id2);
    double y2 = mesh.Cell0DsCoordinates(1, id2);
    double z2 = mesh.Cell0DsCoordinates(2, id2);

    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}
// funzione che prende tutti i punti e i lati ,controlla se chiudono un triangolo e lo crea dopo aver resettato tutti le facce del poliedro.
void CreateTriFace(PolyhedralMesh& mesh,const int& p,const int& q, const int& b, const int& c)
{
    // inizializza le variabili e le flag che servono per il ciclo e calcolare il numero di triangoli da creare
    unsigned int id = 0;
    bool found = false; // flag per controllare se il triangolo è già presente 
    unsigned int faces = 0;
    unsigned int  T = 0;
    T = b*b + b*c + c*c; //formula per calcolare il numero di triangoli da creare
    if (q == 3){faces = 4*T;}
    if (q == 4){faces = 8*T;}
    if (q == 5){faces = 20*T;}   
    //pulisco la mesh dal poligono che c'era prima della triangolazione
    mesh.NumCell2Ds = 0;
    mesh.Cell2DsId.clear();
    mesh.Cell2DsVertices.clear();
    mesh.Cell2DsEdges.clear();
    // vettori dove aggiungere i punti e gli edge che sono connessi al punto generico
    vector<unsigned int> VecIDEdges;
    vector<unsigned int> VecIDVertices;  
   

    // qui prende un punto generico
    for (unsigned int i = 0; i < mesh.NumCell0Ds; ++i) {

        // se il numero di triangoli creati è uguale al numero di triangoli da creare esce
        if (faces == mesh.NumCell2Ds){
            break;
        }
        // Resetta i vettori 
        VecIDVertices.clear();
        VecIDEdges.clear();
        VecIDEdges.reserve(10);
        VecIDVertices.reserve(10);         
        VecIDVertices.push_back(i);
        //Salva i punti che sono connessi al punto generico e l'id del lato
        for (unsigned int j = 0; j < mesh.NumCell1Ds; ++j){         
            if (i == mesh.Cell1DsExtrema(0, j)){
                VecIDVertices.push_back(mesh.Cell1DsExtrema(1, j));
                VecIDEdges.push_back(j);}
            if (i == mesh.Cell1DsExtrema(1, j)){
                VecIDVertices.push_back(mesh.Cell1DsExtrema(0, j));
                VecIDEdges.push_back(j);}
            }
                
        // for che legge il vettore (parto da 1 perchè il primo è il punto generico) e controlla se i punti sono connessi tra di loro
        // se sono connessi crea il triangolo e lo aggiunge alla mesh
        for (unsigned int j = 1; j < VecIDVertices.size() - 1 ; ++j){
            for (unsigned int k = j+1; k < VecIDVertices.size() ; ++k){
                found = false;
                for (unsigned int m=0; m < mesh.NumCell2Ds; ++m){
                    // controlla se il triangolo è già presente                        
                    if (mesh.Cell2DsVertices[m][0] == VecIDVertices[0] && mesh.Cell2DsVertices[m][1] == VecIDVertices[k] && mesh.Cell2DsVertices[m][2] == VecIDVertices[j] ||
                        mesh.Cell2DsVertices[m][0] == VecIDVertices[0] && mesh.Cell2DsVertices[m][1] == VecIDVertices[j] && mesh.Cell2DsVertices[m][2] == VecIDVertices[k] ||
                        mesh.Cell2DsVertices[m][0] == VecIDVertices[j] && mesh.Cell2DsVertices[m][1] == VecIDVertices[0] && mesh.Cell2DsVertices[m][2] == VecIDVertices[k] ||
                        mesh.Cell2DsVertices[m][0] == VecIDVertices[j] && mesh.Cell2DsVertices[m][1] == VecIDVertices[k] && mesh.Cell2DsVertices[m][2] == VecIDVertices[0] ||
                        mesh.Cell2DsVertices[m][0] == VecIDVertices[k] && mesh.Cell2DsVertices[m][1] == VecIDVertices[j] && mesh.Cell2DsVertices[m][2] == VecIDVertices[0] ||
                        mesh.Cell2DsVertices[m][0] == VecIDVertices[k] && mesh.Cell2DsVertices[m][1] == VecIDVertices[0] && mesh.Cell2DsVertices[m][2] == VecIDVertices[j]){
                        found = true;
                        break;}}
                if (found == false){ // se il triangolo non è presente lo crea rispettando l'ordine richiesto
                    for (unsigned int l = 0; l < mesh.NumCell1Ds; ++l){
                        if (mesh.NumCell2Ds == 0){
                            if (mesh.Cell1DsExtrema(0, l) == VecIDVertices[j] && mesh.Cell1DsExtrema(1, l) == VecIDVertices[k]){
                                mesh.Cell2DsId.push_back(id);
                                mesh.NumCell2Ds++;
                                mesh.Cell2DsVertices.push_back({VecIDVertices[0],VecIDVertices[j], VecIDVertices[k]});
                                mesh.Cell2DsEdges.push_back({VecIDEdges[j-1],l, VecIDEdges[k-1]});
                                id++;
                                found = true;}
                            if (mesh.Cell1DsExtrema(0, l) == VecIDVertices[k] && mesh.Cell1DsExtrema(1, l) == VecIDVertices[j] && found == false){
                                mesh.Cell2DsId.push_back(id);
                                mesh.NumCell2Ds++;
                                mesh.Cell2DsVertices.push_back({VecIDVertices[0],VecIDVertices[k], VecIDVertices[j]});
                                mesh.Cell2DsEdges.push_back({VecIDEdges[k-1],l, VecIDEdges[j-1]});
                                id++;}
                        }
                        if (mesh.Cell1DsExtrema(0, l) == VecIDVertices[j] && mesh.Cell1DsExtrema(1, l) == VecIDVertices[k] && found == false){
                            mesh.Cell2DsId.push_back(id);
                            mesh.NumCell2Ds++;
                            mesh.Cell2DsVertices.push_back({VecIDVertices[0],VecIDVertices[j], VecIDVertices[k]});
                            mesh.Cell2DsEdges.push_back({VecIDEdges[j-1],l, VecIDEdges[k-1]});
                            id++;
                            found = true;}
                        if (mesh.Cell1DsExtrema(0, l) == VecIDVertices[k] && mesh.Cell1DsExtrema(1, l) == VecIDVertices[j] && found == false){
                            mesh.Cell2DsId.push_back(id);
                            mesh.NumCell2Ds++;
                            mesh.Cell2DsVertices.push_back({VecIDVertices[0],VecIDVertices[k], VecIDVertices[j]});
                            mesh.Cell2DsEdges.push_back({VecIDEdges[k-1],l, VecIDEdges[j-1]});
                            id++;
                            found = true;}
                    }
                }
            }   
            
        }

    }
    cout << "100%" << endl;
}    


//funzione che controlla se l'edge è già presente nella mesh
// se non lo è lo crea e restituisce il suo id
int GetorCreateEdge(PolyhedralMesh& mesh,unsigned int x,unsigned int y, int l)
{
    unsigned int id;

    for (unsigned int i = 0; i < mesh.NumCell1Ds ; ++i) {
        if ((mesh.Cell1DsExtrema(0, i) == x && mesh.Cell1DsExtrema(1, i) == y) ||
            (mesh.Cell1DsExtrema(0, i) == y && mesh.Cell1DsExtrema(1, i) == x)) {
            return mesh.Cell1DsId[i];
        }
    }
    
    id = mesh.NumCell1Ds;
    mesh.NumCell1Ds++;
    mesh.Cell1DsId.push_back(id);
    mesh.Cell1DsExtrema.conservativeResize(2, mesh.NumCell1Ds); 
    mesh.Cell1DsExtrema(0, id) = x;
    mesh.Cell1DsExtrema(1, id) = y;
    mesh.Cell1DsMarker[0].push_back(id); // marker 0 per gli edge creati 
    
    return id;
}
// funzione che controlla se il punto è già presente nella mesh
// se non lo è lo crea e restituisce il suo id
unsigned int GetorCreatePoint(PolyhedralMesh& mesh,double& x, double& y, double& z, unsigned int l)
{
    unsigned int id;
    vector<unsigned int>::iterator it;
    
    for (unsigned int i = 0; i < mesh.NumCell0Ds; ++i) {
        if (mesh.Cell0DsCoordinates(0, i) == x &&
            mesh.Cell0DsCoordinates(1, i) == y &&
            mesh.Cell0DsCoordinates(2, i) == z) {
                it = find (mesh.Cell2DsVertices[l].begin(), mesh.Cell2DsVertices[l].end(), i);
                    if (it == mesh.Cell2DsVertices[l].end())
                        mesh.Cell2DsVertices[l].push_back({i});
                return mesh.Cell0DsId[i];
        }
    }
    
    id = mesh.NumCell0Ds;
    mesh.NumCell0Ds++;
    mesh.Cell0DsId.push_back(id);
    mesh.Cell0DsCoordinates.conservativeResize(3, mesh.NumCell0Ds);
    mesh.Cell0DsCoordinates(0, id) = x;
    mesh.Cell0DsCoordinates(1, id) = y;
    mesh.Cell0DsCoordinates(2, id) = z;
    mesh.Cell2DsVertices[l].push_back({id});
    mesh.Cell0DsMarker[0].push_back(id); // marker 0 per i punti creati
    
    return id;
}
void Export_Polyhedron(PolyhedralMesh& P)
{
    //Cell0Ds.txt
    ofstream ofile1("Cell0Ds.txt");
    ofile1 << "Id;X;Y;Z\n"; //header 
    
    const MatrixXd &A = P.Cell0DsCoordinates;
    for(unsigned int id=0; id < P.NumCell0Ds; id++)
        ofile1 << defaultfloat << id << ';' << scientific << setprecision(16) << A(0,id) << ';' << A(1,id) << ';' << A(2,id) << '\n';
    ofile1.close();

    //Cell1Ds.txt
    ofstream ofile2("Cell1Ds.txt");
    ofile2 << "Id;Origin;End\n"; //header

    const MatrixXi &B = P.Cell1DsExtrema;
    for(unsigned int id=0; id < P.NumCell1Ds; id++)
        ofile2 << id << ';' << B(0,id) << ';' << B(1,id) << '\n';
    ofile2.close();

    //Cell2Ds.txt
    ofstream ofile3("Cell2Ds.txt");
    ofile3 << "Id;NumVertices;Vertices;NumEdges;Edges\n"; //header
    
   
    const vector<vector<unsigned int>> &V = P.Cell2DsVertices;
    const vector<vector<unsigned int>> &E = P.Cell2DsEdges;
    for(unsigned int id=0; id < P.NumCell2Ds; id++)
    {
        ofile3 << id << ';' << V[id].size();
        for(unsigned int i=0; i < V[id].size(); i++)
            ofile3 << ';' << V[id][i];
        
        ofile3 << ';' << E[id].size();
        for(unsigned int j=0; j < E[id].size(); j++)
            ofile3 << ';' << E[id][j];

        ofile3 << '\n';
    }
    ofile3.close();

    //Cell3Ds.txt
    ofstream ofile4("Cell3Ds.txt");
    ofile4 << "Id;NumVertices;Vertices;NumEdges;Edges;NumFaces;Faces\n";

    ofile4 << 0 << ';' << P.NumCell0Ds;
    for(unsigned int id_vert=0; id_vert < P.NumCell0Ds; id_vert++)
        ofile4 << ';' << id_vert;
    
    ofile4 << ';' << P.NumCell1Ds;
    for(unsigned int id_edge=0; id_edge < P.NumCell1Ds; id_edge++)
        ofile4 << ';' << id_edge;

    ofile4 << ';' << P.NumCell2Ds;
    for(unsigned int id_face=0; id_face < P.NumCell2Ds; id_face++)
        ofile4 << ';' << id_face;
    ofile4 << '\n';
    ofile4.close();
}
    
// funzione che triangola il poliedro 
bool Triangulate(PolyhedralMesh& mesh,const int& p,const int& q, const int& b, const int& c)
{
    if ((b > 1 && c == 0) || (c > 1 && b == 0) || (b == c)) {
        unsigned int d ;        
        if (b==c){
            d = b;
        }
        else{
            d = b + c;
        }
        double eps = 0.00001; // tolleranza per la distanza tra i punti
        unsigned int vert ;
        unsigned int  T = 0;
        T = d*d; 
        if (q == 3){vert = 2*T +2;}
        if (q == 4){vert = 4*T +2;}
        if (q == 5){vert = 10*T +2;} 
        double d_d = d; // conversione in double per il calcolo delle coordinate baricentriche,per evitare la divisione intera
        unsigned int id;
        vector<vector<unsigned int> > Tag; // qui salvo i punti creati con le loro coordinate baricentriche
        vector<unsigned int> TagVec;
        vector<unsigned int> Proximity; // vettore che contiene i punti che sono vicini agli spigoli del triangolo
        unsigned int red; // questi sono i punti della triangolazione I Classe su una faccia triangolare
        for (unsigned int i = 0; i < d+1; ++i) {
            red += i;
        }
        unsigned int numlati = mesh.NumCell1Ds;
        MatrixXd A(3,1), B(3,1), C(3,1),R(3,1);
        double Lato = 0.0; // lunghezza del lato del triangolo
        for (int l=0; l<mesh.NumCell2Ds; ++l) {
            

            Lato = Distance(mesh, mesh.Cell2DsVertices[l][0], mesh.Cell2DsVertices[l][1]);

            A = mesh.Cell0DsCoordinates.col(mesh.Cell2DsVertices[l][0]);
            B = mesh.Cell0DsCoordinates.col(mesh.Cell2DsVertices[l][1]);
            C = mesh.Cell0DsCoordinates.col(mesh.Cell2DsVertices[l][2]);            
            //creazione dei punti per la triangolazione I Classe 
            for (unsigned int i = 0; i <= d; ++i) {
                for (unsigned int j = 0; j <= d-i; ++j) {
                    for (unsigned int k = 0; k <= d-j-i; ++k) {
                        if (i + j + k == d){
                        R = (i/d_d)*A + (j/d_d)*B + (k/d_d)*C;
                        id = GetorCreatePoint(mesh, R(0,0),R(1,0),R(2,0),l);
                        TagVec.push_back(id);
                        TagVec.push_back(i);
                        TagVec.push_back(j);
                        TagVec.push_back(k);
                        Tag.push_back(TagVec);
                        TagVec.clear();
                        TagVec.reserve(4);                        
                        }
                    }
                }
            }
            if (b !=c){
                for (auto it = mesh.Cell2DsVertices[l].begin(); it != mesh.Cell2DsVertices[l].end(); ++it) {
                    for (auto it2 = mesh.Cell2DsVertices[l].begin(); it2 != mesh.Cell2DsVertices[l].end(); ++it2) {
                        if (Distance(mesh, *it, *it2) <= (Lato/d_d + eps) && *it != *it2){ 
                            unsigned int x = *it;
                            unsigned int y = *it2;
                            GetorCreateEdge(mesh,x,y,l);
                        }
                    }
                }
            }
            if ( b == c){
                //funzione che fa i midpoint
                unsigned int i = 0;
                unsigned int j = 0;
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                for (auto it = Tag.begin(); it != Tag.end(); ++it) {
                    for (auto it2 = Tag.begin() ; it2 != Tag.end(); ++it2) {
                        if ((Distance(mesh, (*it)[0], (*it2)[0]) <= (Lato/d_d + eps)) && (*it)[0] != (*it2)[0]){
                            unsigned int id1 = (*it)[0];
                            unsigned int id2 = (*it2)[0];
                            // Seleziono solo i punti che fanno parte dei triangoli rossi per creare i midpoint della triangolazione II Classe
                            for (auto it3 = Tag.begin(); it3 != Tag.end(); ++it3) {
                                unsigned int id3 = (*it3)[0];
                                if ((Distance(mesh, (*it3)[0], (*it2)[0]) <= (Lato/d_d + eps)) && (Distance(mesh, (*it3)[0], (*it)[0]) <= (Lato/d_d + 0.000001)) && (*it3)[0] != (*it2)[0] && (*it3)[0] != (*it)[0]){                    
                                    x = (mesh.Cell0DsCoordinates(0, id1) + mesh.Cell0DsCoordinates(0, id2) + mesh.Cell0DsCoordinates(0, id3)) / 3;
                                    y = (mesh.Cell0DsCoordinates(1, id1) + mesh.Cell0DsCoordinates(1, id2) + mesh.Cell0DsCoordinates(1, id3)) / 3;
                                    z = (mesh.Cell0DsCoordinates(2, id1) + mesh.Cell0DsCoordinates(2, id2) + mesh.Cell0DsCoordinates(2, id3)) / 3;
                                    GetorCreatePoint(mesh,x,y,z,l);
                                    
                                }
                            }
                            // Creo delle booleane pec controllare quali punti appartengono a quale lato e per fare i midpoint
                            bool i1 = (*it)[1] != 0;
                            bool i2 = (*it2)[1] != 0;
                            bool j1 = (*it)[2] != 0;
                            bool j2 = (*it2)[2] != 0;
                            bool k1 = (*it)[3] != 0;
                            bool k2 = (*it2)[3] != 0;
                            bool inter1 = i1 && j1 && k1;
                            bool inter2 = i2 && j2 && k2;
                            // qui gestisco i punti laterali che appartengono allo stesso lato 
                            if ((i1 == i2 && j1 == j2 && k1 == k2) && (!i1 || !j1 || !k1) && (i1 || j1 || k1)){
                                x = (mesh.Cell0DsCoordinates(0, id1) + mesh.Cell0DsCoordinates(0, id2)) / 2;
                                y = (mesh.Cell0DsCoordinates(1, id1) + mesh.Cell0DsCoordinates(1, id2)) / 2;
                                z = (mesh.Cell0DsCoordinates(2, id1) + mesh.Cell0DsCoordinates(2, id2)) / 2;
                                GetorCreatePoint(mesh,x,y,z,l);
                            }
                            // qui gestisco i punti laterali che sono vicini ai vertici del triangolo
                            if ((i1 != i2 && j1 == j2 && k1 == k2 && !inter1 && !inter2) || (i1 == i2 && j1 != j2 && k1 == k2 && !inter1 && !inter2) || (i1 == i2 && j1 == j2 && k1 != k2 && !inter1 && !inter2)){
                                x = (mesh.Cell0DsCoordinates(0, id1) + mesh.Cell0DsCoordinates(0, id2)) / 2;
                                y = (mesh.Cell0DsCoordinates(1, id1) + mesh.Cell0DsCoordinates(1, id2)) / 2;
                                z = (mesh.Cell0DsCoordinates(2, id1) + mesh.Cell0DsCoordinates(2, id2)) / 2;
                                Proximity.push_back(GetorCreatePoint(mesh,x,y,z,l));

                            }
                        }
                    }
                }
                // prendo i punti che sono vicini e creo i lati
                for (auto it = mesh.Cell2DsVertices[l].begin(); it != mesh.Cell2DsVertices[l].end(); ++it) {
                    for (auto it2 = mesh.Cell2DsVertices[l].begin(); it2 != mesh.Cell2DsVertices[l].end(); ++it2) {
                        if (Distance(mesh, *it, *it2) <= (Lato/(d_d*sqrt(3)) + eps) && (*it != *it2) && !(find(Proximity.begin(), Proximity.end(), *it) != Proximity.end() && find(Proximity.begin(), Proximity.end(), *it2) != Proximity.end())){ 
                            unsigned int x = *it;
                            unsigned int y = *it2;
                            GetorCreateEdge(mesh,x,y,l);
                        }
                    }
                }
                
            }
            // Pulisco i vettori e alloco la memoria necessaria
            Tag.clear();
            Tag.reserve(red);
            Proximity.clear();
            Proximity.reserve(6);
        }     
        // risistemo gli id degli edge e dei punti
        for (unsigned int i = 0;i < numlati; ++i) {
            mesh.Cell1DsId.erase(mesh.Cell1DsId.begin() + i);
            mesh.NumCell1Ds--;
        }
        for (unsigned int i = 0;i < mesh.NumCell1Ds; ++i){
            mesh.Cell1DsId[i] = i ;
        }
        mesh.Cell1DsExtrema = mesh.Cell1DsExtrema.rightCols(mesh.NumCell1Ds).eval();        
        CreateTriFace(mesh,p,q,b,c);
    }    
    return true;
}




// funzione che centralizza la mesh
bool Centralize(PolyhedralMesh& mesh)
{
    if (mesh.NumCell0Ds == 0) return false;


    double xsum;
    double ysum;
    double zsum;
    // calcolo il baricentro della mesh
    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
        xsum += mesh.Cell0DsCoordinates(0, i);
        ysum += mesh.Cell0DsCoordinates(1, i);
        zsum += mesh.Cell0DsCoordinates(2, i);
    }


    // Sposto il baricentro in (0,0,0)
    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
        mesh.Cell0DsCoordinates(0, i) -= (xsum / mesh.NumCell0Ds);
        mesh.Cell0DsCoordinates(1, i) -= (ysum / mesh.NumCell0Ds);
        mesh.Cell0DsCoordinates(2, i) -= (zsum / mesh.NumCell0Ds);
    }
    return true;
}
// funzione che proietta i punti della mesh sulla sfera unitaria
bool Normalize(PolyhedralMesh& mesh)
{
    if (mesh.NumCell0Ds == 0) return false;
    double norm;
    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
        norm = sqrt(
            mesh.Cell0DsCoordinates(0, i) * mesh.Cell0DsCoordinates(0, i) +
            mesh.Cell0DsCoordinates(1, i) * mesh.Cell0DsCoordinates(1, i) +
            mesh.Cell0DsCoordinates(2, i) * mesh.Cell0DsCoordinates(2, i)  );
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
    vector<unsigned int> markers;
    vector<double> xs, ys, zs;
    while (getline(file, line)) {
        stringstream ss(line);
        unsigned int id, marker;
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
    for (unsigned int i = 0; i < mesh.NumCell0Ds; ++i) {
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
    for (unsigned int i = 0; i < mesh.NumCell1Ds; ++i) {
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
        unsigned int id,numV, numE;
        char sep;
        ss >> id >> sep >> numV;
        vector<unsigned int> verts(numV);
        for (int i = 0; i < numV; ++i) ss >> sep >> verts[i];
        ss >> sep >> numE;
        vector<unsigned int> edges(numE);
        for (int i = 0; i < numE; ++i) ss >> sep >> edges[i];

        mesh.Cell2DsId.push_back(id);
        mesh.Cell2DsVertices.push_back(move(verts));
        mesh.Cell2DsEdges.push_back(move(edges));
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