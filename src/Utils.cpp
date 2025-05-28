
#include "Utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include "UCDUtilities.hpp"

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
    bool NG = false; 
    bool found = false; // flag per controllare se il triangolo è già presente 
    unsigned int faces = 0;
    unsigned int  T = 0;
    double lato = 0.0; // lunghezza del lato del triangolo
    double d_b = b + c; // conversione in double per evitare la divisione intera
    lato = Distance(mesh, 0, 1); // prendo la lunghezza del lato del triangolo
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
    //bug fix:nel caso  3 3 b != c esistono facce triangolari che sono interne al poliedro ,precisamente sono triangoli che sono vicini al vertice del tetraedro     
    vector<vector<unsigned int>> VecNG; // vettore che contiene i punti che sono vicini al vertice del tetraedro
    VecNG.reserve(4);
    vector<unsigned int> NG0, NG1, NG2, NG3;    
    if (p==3 && q==3 && b!=c) {
        for (unsigned int i = 0; i < mesh.NumCell0Ds; ++i) {
            if (Distance(mesh, i, 0) <= lato/d_b + 0.000001 && i != 0){
                NG0.push_back({i});
            }
            if (Distance(mesh, i, 1) <= lato/d_b + 0.000001 && i != 1){
                NG1.push_back({i});
            }
            if (Distance(mesh, i, 2) <= lato/d_b + 0.000001 && i != 2){
                NG2.push_back({i});
            }
            if (Distance(mesh, i, 3) <= lato/d_b + 0.000001 && i != 3){
                NG3.push_back({i});
            }
        }
        VecNG.push_back(NG0);
        VecNG.push_back(NG1);
        VecNG.push_back(NG2);
        VecNG.push_back(NG3);
    }

    // qui prende un punto generico
    for (unsigned int i = 0; i < mesh.NumCell0Ds; ++i) {

        // se il numero di triangoli creati è uguale al numero di triangoli da creare esce
        //if (faces == mesh.NumCell2Ds){
        //    break;
        //}
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
                NG = false; // resetto la flag per il triangolo
                if (p==3 && q==3 && b!=c){ // caso tetraedro con triangoli interni
                    for (unsigned int m = 0; m < 4; ++m) {
                        if (VecNG[m][0] == VecIDVertices[j] && VecNG[m][1] == VecIDVertices[k] && VecNG[m][2] == VecIDVertices[0] ||
                            VecNG[m][0] == VecIDVertices[k] && VecNG[m][1] == VecIDVertices[j] && VecNG[m][2] == VecIDVertices[0] || 
                            VecNG[m][0] == VecIDVertices[0] && VecNG[m][1] == VecIDVertices[j] && VecNG[m][2] == VecIDVertices[k] ||
                            VecNG[m][0] == VecIDVertices[0] && VecNG[m][1] == VecIDVertices[k] && VecNG[m][2] == VecIDVertices[j] ||
                            VecNG[m][0] == VecIDVertices[j] && VecNG[m][1] == VecIDVertices[0] && VecNG[m][2] == VecIDVertices[k] ||
                            VecNG[m][0] == VecIDVertices[k] && VecNG[m][1] == VecIDVertices[0] && VecNG[m][2] == VecIDVertices[j]){
                            
                            NG = true; // se il punto è vicino al vertice del tetraedro allora lo considero
                            break;
                        }
                    }
                }
                if (NG == false){
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
    }
    cout << "100%" << endl;
}    


//funzione che controlla se l'edge è già presente nella mesh
// se non lo è lo crea e restituisce il suo id
unsigned int GetorCreateEdge(PolyhedralMesh& mesh,unsigned int& x,unsigned int& y, unsigned int l)
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
    double epsilon = 1e-6;    // tolleranza per la comparazione dei punti 
    for (unsigned int i = 0; i < mesh.NumCell0Ds; ++i) {

        if (abs(mesh.Cell0DsCoordinates(0, i) - x) < epsilon &&
            abs(mesh.Cell0DsCoordinates(1, i) - y) < epsilon &&
            abs(mesh.Cell0DsCoordinates(2, i) - z) < epsilon) {
                it = find (mesh.Cell2DsVertices[l].begin(), mesh.Cell2DsVertices[l].end(), i); // da aggiustare perchè se il punto è alla fine lo da come non trovato
                    if (it == mesh.Cell2DsVertices[l].end() && *it != i) { //così dovrebbe andare
                        mesh.Cell2DsVertices[l].push_back({i});}
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

    if ((b > 1 && c == 0) || (c > 1 && b == 0) || (b == c && b > 1 && c > 1)) {
        unsigned int d ;
        unsigned int numlati = mesh.NumCell1Ds;            
        if (b==c){
            d = b;
        }
        else{
            d = b + c;
        }
        double eps = 1e-6; // tolleranza per la distanza tra i punti
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
    if (b == c && b == 1){
        unsigned int numlati = mesh.NumCell1Ds;   
        for (unsigned int i = 0; i < mesh.NumCell2Ds; ++i) {
            MatrixXd A(3,1), B(3,1), C(3,1),R(3,1);
            unsigned int iA = mesh.Cell2DsVertices[i][0];
            unsigned int iB = mesh.Cell2DsVertices[i][1];
            unsigned int iC = mesh.Cell2DsVertices[i][2];
            A = mesh.Cell0DsCoordinates.col(iA).eval();
            B = mesh.Cell0DsCoordinates.col(iB).eval();
            C = mesh.Cell0DsCoordinates.col(iC).eval(); 
            R = (A + B + C) / 3; // calcolo il baricentro della faccia triangolare
            unsigned int idABC = GetorCreatePoint(mesh, R(0,0), R(1,0), R(2,0), i);
            R = (A + B) / 2.;
            unsigned int idAB = GetorCreatePoint(mesh, R(0,0), R(1,0), R(2,0), i);
            R = (B + C) / 2.;
            unsigned int idBC = GetorCreatePoint(mesh, R(0,0), R(1,0), R(2,0), i);
            R = (C + A) / 2.;
            unsigned int idCA = GetorCreatePoint(mesh, R(0,0), R(1,0), R(2,0), i);
            // creo gli edge tra i punti del triangolo e il baricentro
            GetorCreateEdge(mesh, idABC, idAB, i);
            GetorCreateEdge(mesh, idABC, idBC, i);
            GetorCreateEdge(mesh, idABC, idCA, i);
            // creo gli edge tra i punti del triangolo e i punti medi
            GetorCreateEdge(mesh, idABC, iA, i);
            GetorCreateEdge(mesh, idABC, iB, i);
            GetorCreateEdge(mesh, idABC, iC, i);
            // creo gli edge tra i punti medi
            GetorCreateEdge(mesh, idAB, iA, i);
            GetorCreateEdge(mesh, idAB, iB, i);
            GetorCreateEdge(mesh, idBC, iB, i);
            GetorCreateEdge(mesh, idBC, iC, i);
            GetorCreateEdge(mesh, idCA, iA, i);
            GetorCreateEdge(mesh, idCA, iC, i);                
            }
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

void CalculateFaceCentroids(const PolyhedralMesh& originalMesh, PolyhedralMesh& dualMesh)
{
    cout << "Calcolando baricentri" << endl;
    
    // Inizializza la struttura dei vertici del duale
    dualMesh.NumCell0Ds = originalMesh.NumCell2Ds;  // Tanti vertici quante facce originali
    dualMesh.Cell0DsId.resize(dualMesh.NumCell0Ds);
    dualMesh.Cell0DsCoordinates.resize(3, dualMesh.NumCell0Ds);
    
    // Per ogni faccia del poliedro originale
    for (unsigned int faceId = 0; faceId < originalMesh.NumCell2Ds; ++faceId) {
        
        // Calcola il baricentro della faccia
        double centroidX = 0.0, centroidY = 0.0, centroidZ = 0.0;
        const vector<unsigned int>& faceVertices = originalMesh.Cell2DsVertices[faceId];
        
        // Somma le coordinate di tutti i vertici della faccia
        for (unsigned int vertexId : faceVertices) {
            centroidX += originalMesh.Cell0DsCoordinates(0, vertexId);
            centroidY += originalMesh.Cell0DsCoordinates(1, vertexId);
            centroidZ += originalMesh.Cell0DsCoordinates(2, vertexId);
        }
        
        // Media per ottenere il baricentro
        unsigned int numVertices = faceVertices.size();
        centroidX /= numVertices;
        centroidY /= numVertices;
        centroidZ /= numVertices;
        
        // Aggiungi il baricentro come nuovo vertice del duale
        dualMesh.Cell0DsId[faceId] = faceId;
        dualMesh.Cell0DsCoordinates(0, faceId) = centroidX;
        dualMesh.Cell0DsCoordinates(1, faceId) = centroidY;
        dualMesh.Cell0DsCoordinates(2, faceId) = centroidZ;
    }
    
    cout << "Created " << dualMesh.NumCell0Ds << " centroid vertices for dual mesh." << endl;
}
void CreateDualEdges(const PolyhedralMesh& originalMesh, PolyhedralMesh& dualMesh)
{
    cout << "Creazione spigoli duali" << endl;
    
    vector<unsigned int> dualEdgeIds;
    vector<pair<unsigned int, unsigned int>> dualEdgeExtrema;
    
    // Per ogni edge del poliedro originale
    for (unsigned int edgeId = 0; edgeId < originalMesh.NumCell1Ds; ++edgeId) {
        
        // Trova le due facce che condividono questo edge
        vector<unsigned int> adjacentFaces;
        
        for (unsigned int faceId = 0; faceId < originalMesh.NumCell2Ds; ++faceId) {
            const vector<unsigned int>& faceEdges = originalMesh.Cell2DsEdges[faceId];
            int i = 0;
            // Controlla se questo edge appartiene alla faccia corrente
            vector<unsigned int>::const_iterator it = originalMesh.Cell2DsEdges[faceId].begin();
            vector<unsigned int>::const_iterator end = originalMesh.Cell2DsEdges[faceId].end();
            while (it != end && i < 2) {
                unsigned int currentEdgeId = *it;
                if (currentEdgeId == edgeId) {
                    adjacentFaces.push_back(faceId);
                    i++;
                    //cout << "Edge " << edgeId << " is part of face " << faceId << "." << endl;
                    if( i == 2){
                        break;
                    }
                }
                ++it;
            }
            /*if (find(originalMesh.Cell2DsEdges[faceId].begin(), originalMesh.Cell2DsEdges[faceId].end(), edgeId) != originalMesh.Cell2DsEdges[faceId].end() || *originalMesh.Cell2DsEdges[faceId].end() == edgeId) {
                adjacentFaces.push_back(faceId);
                i++;
                //cout << "Edge " << edgeId << " is part of face " << faceId << "." << endl;
                if( i == 2){
                break;}
            }*/
        }
        if (adjacentFaces.size() != 2) {
            cout << "Warning: Edge " << edgeId << " is not shared by any faces." << endl;
            continue; // Se l'edge non è condiviso da nessuna faccia, lo saltiamo
        }
        //cout << "Edge " << edgeId << " is shared by " << adjacentFaces.size() << " faces." << endl;
        // Ogni edge deve essere condiviso da esattamente 2 facce
        if (adjacentFaces.size() == 2) {
            unsigned int face1 = adjacentFaces[0];
            unsigned int face2 = adjacentFaces[1];
            
            // Crea edge nel duale che connette i baricentri delle due facce
            dualEdgeIds.push_back(edgeId);
            dualEdgeExtrema.push_back({face1, face2});
        }
    }
    
    // Popola la struttura del duale
    dualMesh.NumCell1Ds = dualEdgeIds.size();
    dualMesh.Cell1DsId = dualEdgeIds;
    dualMesh.Cell1DsExtrema.resize(2, dualMesh.NumCell1Ds);
    
    for (unsigned int i = 0; i < dualMesh.NumCell1Ds; ++i) {
        dualMesh.Cell1DsExtrema(0, i) = dualEdgeExtrema[i].first;
        dualMesh.Cell1DsExtrema(1, i) = dualEdgeExtrema[i].second;
    }
    
    cout << "Created " << dualMesh.NumCell1Ds << " edges for dual mesh." << endl;
}
vector<unsigned int> OrderDualFaceVertices(const vector<unsigned int>& faceVertices, const PolyhedralMesh& dualMesh)   //ordinamento corretto
{
    vector<unsigned int> ordered;
    if (faceVertices.empty()) return ordered;
    
    // Calcola il baricentro della faccia duale
    Vector3d baricenter = Vector3d::Zero();
    for (auto vertexId : faceVertices) {
        baricenter(0) += dualMesh.Cell0DsCoordinates(0, vertexId);
        baricenter(1) += dualMesh.Cell0DsCoordinates(1, vertexId);
        baricenter(2) += dualMesh.Cell0DsCoordinates(2, vertexId);
    }
    baricenter /= faceVertices.size();
    
    // Calcola la normale al piano della faccia usando tre punti
    Vector3d p0(dualMesh.Cell0DsCoordinates(0, faceVertices[0]), 
                dualMesh.Cell0DsCoordinates(1, faceVertices[0]), 
                dualMesh.Cell0DsCoordinates(2, faceVertices[0]));
    Vector3d p1(dualMesh.Cell0DsCoordinates(0, faceVertices[1]), 
                dualMesh.Cell0DsCoordinates(1, faceVertices[1]), 
                dualMesh.Cell0DsCoordinates(2, faceVertices[1]));
    Vector3d p2(dualMesh.Cell0DsCoordinates(0, faceVertices[2]), 
                dualMesh.Cell0DsCoordinates(1, faceVertices[2]), 
                dualMesh.Cell0DsCoordinates(2, faceVertices[2]));
    
    Vector3d v1 = p1 - p0;
    Vector3d v2 = p2 - p0;
    Vector3d normal = v1.cross(v2).normalized();
    
    // Direzione di riferimento dal baricentro al primo punto
    Vector3d ref_dir = (p0 - baricenter).normalized();
    
    // Sistema di coordinate locali sul piano
    Vector3d u_axis = ref_dir;
    Vector3d v_axis = normal.cross(u_axis).normalized();
    
    // Calcola l'angolo per ogni punto
    vector<pair<double, unsigned int>> angle_id_pairs;
    for (auto vertexId : faceVertices) {
        Vector3d point(dualMesh.Cell0DsCoordinates(0, vertexId),
                      dualMesh.Cell0DsCoordinates(1, vertexId),
                      dualMesh.Cell0DsCoordinates(2, vertexId));
        Vector3d vec = point - baricenter;
        double x = vec.dot(u_axis);
        double y = vec.dot(v_axis);
        double angle = atan2(y, x);
        
        // Correggi l'angolo per essere tra 0 e 2π
        if (angle < 0) angle += 2 * M_PI;
        
        angle_id_pairs.emplace_back(angle, vertexId);
    }
    
    // Ordina per angolo crescente
    sort(angle_id_pairs.begin(), angle_id_pairs.end());
    
    // Estrai gli ID ordinati
    for (const auto& pair : angle_id_pairs) {
        ordered.push_back(pair.second);
    }
    
    return ordered;
}
void CreateDualFaces(const PolyhedralMesh& originalMesh, PolyhedralMesh& dualMesh)
{
    cout << "Creazione facce duali" << endl;
    
    // Per ogni vertice del poliedro originale
    for (unsigned int vertexId = 0; vertexId < originalMesh.NumCell0Ds; ++vertexId) {
        
        // Trova tutte le facce incidenti a questo vertice
        vector<unsigned int> incidentFaces;
        
        for (unsigned int faceId = 0; faceId < originalMesh.NumCell2Ds; ++faceId) {
            const vector<unsigned int>& faceVertices = originalMesh.Cell2DsVertices[faceId];
            
            // Controlla se il vertice appartiene a questa faccia
            if (find(faceVertices.begin(), faceVertices.end(), vertexId) != faceVertices.end() || *faceVertices.end() == vertexId) {
                incidentFaces.push_back(faceId);
            }
        }
        
        // Crea una faccia nel duale usando i baricentri delle facce incidenti
        if (incidentFaces.size() >= 3) {  // Una faccia deve avere almeno 3 vertici
            
            
            // Ordina correttamente i vertici della faccia duale
            vector<unsigned int> orderedVertices = OrderDualFaceVertices(incidentFaces, dualMesh);
            
            dualMesh.Cell2DsId.push_back(vertexId);
            dualMesh.Cell2DsVertices.push_back(orderedVertices);  // <-- usa orderedVertices invece di incidentFaces
            
            
            vector<unsigned int> faceEdges;
            
            dualMesh.Cell2DsEdges.push_back(faceEdges);
        }
    }
    
    dualMesh.NumCell2Ds = dualMesh.Cell2DsId.size();
    cout << "Create " << dualMesh.NumCell2Ds << " facce per mesh duale." << endl;
}
void ProjectDualToSphere(PolyhedralMesh& dualMesh)
{
    cout << "Proietto la mesh duale sulla sfera unitaria..." << endl;
    
    if (dualMesh.NumCell0Ds == 0) {
        cerr << "Nessun vertice da proiettare sulla mesh duale" << endl;
        return;
    }
    
    // Per ogni vertice del duale
    for (unsigned int i = 0; i < dualMesh.NumCell0Ds; ++i) {
        
        // Calcola la norma del punto
        double x = dualMesh.Cell0DsCoordinates(0, i);
        double y = dualMesh.Cell0DsCoordinates(1, i);
        double z = dualMesh.Cell0DsCoordinates(2, i);
        
        double norm = sqrt(x*x + y*y + z*z);
        
        // Proietta sulla sfera unitaria
        if (norm > 0) {  // Evita divisioni per zero
            dualMesh.Cell0DsCoordinates(0, i) = x / norm;
            dualMesh.Cell0DsCoordinates(1, i) = y / norm;
            dualMesh.Cell0DsCoordinates(2, i) = z / norm;
        }
    }
    
    cout << "Proiettato " << dualMesh.NumCell0Ds << "vertici duali sulla sfera." << endl;
}
void ExportDualFiles(const PolyhedralMesh& dualMesh)
{
    cout << "esporto file duali" << endl;
    
    // Export dei punti del duale per Paraview
    Gedim::UCDUtilities utilities;
    
    utilities.ExportPoints("./dualCell0Ds.inp",
                           dualMesh.Cell0DsCoordinates);
    
    utilities.ExportSegments("./dualCell1Ds.inp",
                             dualMesh.Cell0DsCoordinates,
                             dualMesh.Cell1DsExtrema);
    
    //utilities.ExportPolygons("./dualCell2Ds.inp",
                              //dualMesh.Cell0DsCoordinates,
                             // dualMesh.Cell2DsVertices);
    
    cout << "dile duali esportati: dualCell0Ds.inp, dualCell1Ds.inp, dualCell2Ds.inp" << endl;
}
bool CalculateAndExportDual(PolyhedralMesh& mesh, const int& p, const int& q, const int& b, const int& c)
{
    PolyhedralMesh dualMesh;
    
    cout << "Step 1: calcolo dei baricentri..." << endl;
    CalculateFaceCentroids(mesh, dualMesh);
    
    cout << "Step 2: Creo spigoli duali..." << endl;
    CreateDualEdges(mesh, dualMesh);
    
    cout << "Step 3: creo facce duali..." << endl;
    CreateDualFaces(mesh, dualMesh);
    cout << "Step 4: proietto sulla sfera..." << endl;
    ProjectDualToSphere(dualMesh);
    
     cout << "Step 5: esporto i file..." << endl;
    ExportDualFiles(dualMesh);
    
    cout << "calcolo del duale ed esportazione avvenuti con successo!" << endl;
    
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
        mesh.Cell0DsCoordinates(0, i) /= norm ;
        mesh.Cell0DsCoordinates(1, i) /= norm ;
        mesh.Cell0DsCoordinates(2, i) /= norm ;
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