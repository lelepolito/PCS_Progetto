
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
void CreateTriFace(PolyhedralMesh& mesh,const int& p,const int& q, const int& b, const int& c)
{
    unsigned int id = 0;
    mesh.NumCell2Ds = 0;
    mesh.Cell2DsId.clear();
    mesh.Cell2DsVertices.clear();
    mesh.Cell2DsEdges.clear();
    mesh.Cell2DsMarker.clear();
    vector<unsigned int> VecIDEdges;
    vector<unsigned int> VecIDVertices;  
    VecIDEdges.reserve(10);
    VecIDVertices.reserve(10);    
    bool found = false;
    int faces = 0;
    int T = 0;
    T = b*b + b*c + c*c;
    if (q == 3){faces = 4*T;}
    if (q == 4){faces = 8*T;}
    if (q == 5){faces = 20*T;}

    for (unsigned int i = 0; i < mesh.NumCell0Ds; ++i) {
        VecIDVertices.clear();
        VecIDEdges.clear();
        VecIDVertices.push_back(i);
        for (unsigned int j = 0; j < mesh.NumCell1Ds; ++j){         
            if (i == mesh.Cell1DsExtrema(0, j)){
                VecIDVertices.push_back(mesh.Cell1DsExtrema(1, j));
                VecIDEdges.push_back(j);}
            if (i == mesh.Cell1DsExtrema(1, j)){
                VecIDVertices.push_back(mesh.Cell1DsExtrema(0, j));
                VecIDEdges.push_back(j);}
            }
        if (mesh.NumCell2Ds < faces){        
            cout << VecIDVertices[0]<<"/"<<mesh.NumCell0Ds << "%" << endl;
            for (unsigned int j = 1; j < VecIDVertices.size() - 1 ; ++j){
                for (unsigned int k = j+1; k < VecIDVertices.size() ; ++k){
                    found = false;
                    for (unsigned int m=0; m < mesh.NumCell2Ds; ++m){                       
                        if (mesh.Cell2DsVertices[m][0] == VecIDVertices[0] && mesh.Cell2DsVertices[m][1] == VecIDVertices[k] && mesh.Cell2DsVertices[m][2] == VecIDVertices[j] ||
                            mesh.Cell2DsVertices[m][0] == VecIDVertices[0] && mesh.Cell2DsVertices[m][1] == VecIDVertices[j] && mesh.Cell2DsVertices[m][2] == VecIDVertices[k] ||
                            mesh.Cell2DsVertices[m][0] == VecIDVertices[j] && mesh.Cell2DsVertices[m][1] == VecIDVertices[0] && mesh.Cell2DsVertices[m][2] == VecIDVertices[k] ||
                            mesh.Cell2DsVertices[m][0] == VecIDVertices[j] && mesh.Cell2DsVertices[m][1] == VecIDVertices[k] && mesh.Cell2DsVertices[m][2] == VecIDVertices[0] ||
                            mesh.Cell2DsVertices[m][0] == VecIDVertices[k] && mesh.Cell2DsVertices[m][1] == VecIDVertices[j] && mesh.Cell2DsVertices[m][2] == VecIDVertices[0] ||
                            mesh.Cell2DsVertices[m][0] == VecIDVertices[k] && mesh.Cell2DsVertices[m][1] == VecIDVertices[0] && mesh.Cell2DsVertices[m][2] == VecIDVertices[j]){
                            found = true;
                            //cout << "Face already present" << endl;
                            break;}}
                    if (found == false){
                        for (unsigned int l = 0; l < mesh.NumCell1Ds; ++l){
                            if (mesh.NumCell2Ds == 0){
                                if (mesh.Cell1DsExtrema(0, l) == VecIDVertices[j] && mesh.Cell1DsExtrema(1, l) == VecIDVertices[k]){
                                    mesh.Cell2DsId.push_back(id);
                                    mesh.NumCell2Ds++;
                                    mesh.Cell2DsVertices.push_back({VecIDVertices[0],VecIDVertices[j], VecIDVertices[k]});
                                    mesh.Cell2DsEdges.push_back({VecIDEdges[j-1],l, VecIDEdges[k-1]});
                                    mesh.Cell2DsMarker[0].push_back(id); // marker 0 per le facce create
                                    id++;}
                                if (mesh.Cell1DsExtrema(0, l) == VecIDVertices[k] && mesh.Cell1DsExtrema(1, l) == VecIDVertices[j]){
                                    mesh.Cell2DsId.push_back(id);
                                    mesh.NumCell2Ds++;
                                    mesh.Cell2DsVertices.push_back({VecIDVertices[0],VecIDVertices[k], VecIDVertices[j]});
                                    mesh.Cell2DsEdges.push_back({VecIDEdges[k-1],l, VecIDEdges[j-1]});
                                    mesh.Cell2DsMarker[0].push_back(id); // marker 0 per le facce create
                                    id++;}
                            }
                            if (mesh.Cell1DsExtrema(0, l) == VecIDVertices[j] && mesh.Cell1DsExtrema(1, l) == VecIDVertices[k] && found == false){
                                mesh.Cell2DsId.push_back(id);
                                mesh.NumCell2Ds++;
                                mesh.Cell2DsVertices.push_back({VecIDVertices[0],VecIDVertices[j], VecIDVertices[k]});
                                mesh.Cell2DsEdges.push_back({VecIDEdges[j-1],l, VecIDEdges[k-1]});
                                mesh.Cell2DsMarker[0].push_back(id); // marker 0 per le facce create
                                cout << "Face created " << id << endl;                        
                                id++;
                                found = true;}
                            if (mesh.Cell1DsExtrema(0, l) == VecIDVertices[k] && mesh.Cell1DsExtrema(1, l) == VecIDVertices[j] && found == false){
                                mesh.Cell2DsId.push_back(id);
                                mesh.NumCell2Ds++;
                                mesh.Cell2DsVertices.push_back({VecIDVertices[0],VecIDVertices[k], VecIDVertices[j]});
                                mesh.Cell2DsEdges.push_back({VecIDEdges[k-1],l, VecIDEdges[j-1]});
                                mesh.Cell2DsMarker[0].push_back(id); // marker 0 per le facce create
                                cout << "Face created " << id << endl;
                                id++;
                                found = true;}
                        }
                    }
                }   
            }
        }
    }
}    



int GetorCreateEdge(PolyhedralMesh& mesh,unsigned int x,unsigned int y, int l)
{
    unsigned int id;
    for (size_t i = 0; i < mesh.NumCell1Ds ; ++i) {
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

unsigned int GetorCreateVec(PolyhedralMesh& mesh,double& x, double& y, double& z, unsigned int l)
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
    

bool Triangulate(PolyhedralMesh& mesh,const int& p,const int& q, const int& b, const int& c)
{
    if (b > 1 && c == 0 || c > 1 && b == 0) {
        unsigned int d = b + c;
        double d_d = d;
        unsigned int numlati = mesh.NumCell1Ds;
        MatrixXd A(3,1), B(3,1), C(3,1),R(3,1);
        double Lato = 0.0;
        for (int l=0; l<mesh.NumCell2Ds; ++l) {
            
            vector<unsigned int> vec = mesh.Cell2DsVertices[l];
            vector<unsigned int> vec2 = mesh.Cell2DsEdges[l];
            Lato = Distance(mesh, vec[0], vec[1]);
            
            A = mesh.Cell0DsCoordinates.col(vec[0]);
            B = mesh.Cell0DsCoordinates.col(vec[1]);
            C = mesh.Cell0DsCoordinates.col(vec[2]);            
            
            for (int i = 0; i <= d; ++i) {
                for (int j = 0; j <= d-i; ++j) {
                    for (int k = 0; k <= d-j-i; ++k) {
                        if (i + j + k == d){
                        R = (i/d_d)*A + (j/d_d)*B + (k/d_d)*C;
                        GetorCreateVec(mesh, R(0,0),R(1,0) , R(2,0),l);
                        }
                    }

                }
            }

            for (auto it = mesh.Cell2DsVertices[l].begin(); it != mesh.Cell2DsVertices[l].end(); ++it) {
                for (auto it2 = mesh.Cell2DsVertices[l].begin(); it2 != mesh.Cell2DsVertices[l].end(); ++it2) {
                    if (Distance(mesh, *it, *it2) <= (Lato/d_d + 0.001 ) && *it != *it2){ 
                        unsigned int x = *it;
                        unsigned int y = *it2;
                        GetorCreateEdge(mesh,x,y,l);
                    }
                }
            }
        }     

        for (int i = 0;i < numlati; ++i) {
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





bool Centralize(PolyhedralMesh& mesh)
{
    if (mesh.NumCell0Ds == 0) return false;


    double xsum;
    double ysum;
    double zsum;

    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
        xsum += mesh.Cell0DsCoordinates(0, i);
        ysum += mesh.Cell0DsCoordinates(1, i);
        zsum += mesh.Cell0DsCoordinates(2, i);
    }



    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
        mesh.Cell0DsCoordinates(0, i) -= (xsum / mesh.NumCell0Ds);
        mesh.Cell0DsCoordinates(1, i) -= (ysum / mesh.NumCell0Ds);
        mesh.Cell0DsCoordinates(2, i) -= (zsum / mesh.NumCell0Ds);
    }
    return true;
}
bool Normalize(PolyhedralMesh& mesh)
{
    //mesh.Cell0DsCoordinates = mesh.Cell0DsCoordinates * 1e5; 
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
        //mesh.Cell0DsCoordinates(0, i) *= 1e2;
        //mesh.Cell0DsCoordinates(1, i) *= 1e2;
        //mesh.Cell0DsCoordinates(2, i) *= 1e2;
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
    vector<int> markers;
    vector<double> xs, ys, zs;
    while (getline(file, line)) {
        stringstream ss(line);
        unsigned int id; int marker;
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
    for (size_t i = 0; i < mesh.NumCell0Ds; ++i) {
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
    for (size_t i = 0; i < mesh.NumCell1Ds; ++i) {
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
        unsigned int id; int marker, numV, numE;
        char sep;
        ss >> id >> sep >> marker >> sep >> numV;
        vector<unsigned int> verts(numV);
        for (int i = 0; i < numV; ++i) ss >> sep >> verts[i];
        ss >> sep >> numE;
        vector<unsigned int> edges(numE);
        for (int i = 0; i < numE; ++i) ss >> sep >> edges[i];

        mesh.Cell2DsId.push_back(id);
        mesh.Cell2DsVertices.push_back(move(verts));
        mesh.Cell2DsEdges.push_back(move(edges));
        mesh.Cell2DsMarker[marker].push_back(id);
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