#pragma once

#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <Eigen/Dense>
#include "Utils.hpp"
#include "Eigen/Eigen"
#include "struttura.hpp"

using namespace Eigen;
using namespace std;
using namespace PolygonalLibrary;


TEST(TestPolyhedron, TestNormalize1) {
    PolyhedralMesh mesh;
    mesh.NumCell0Ds = 3;
    mesh.Cell0DsId = {0, 1, 2};
    mesh.Cell0DsCoordinates.resize(3, 3);

    mesh.Cell0DsCoordinates.col(0) = Vector3d(0.0, 0.0, 0.0);  
    mesh.Cell0DsCoordinates.col(1) = Vector3d(1.0, 1.0, 1.0);  
    mesh.Cell0DsCoordinates.col(2) = Vector3d(0.0, 3.0, 4.0);  

    EXPECT_TRUE(Normalize(mesh));

    
    Vector3d v0 = mesh.Cell0DsCoordinates.col(0);
    EXPECT_DOUBLE_EQ(v0.norm(), 0.0);

    for (int i = 1; i < 3; ++i) {
        Vector3d v = mesh.Cell0DsCoordinates.col(i);
        EXPECT_NEAR(v.norm(), 1.0, 1e-6);
    }
}
TEST(TestPolyhedron, TestNormalize2) {
    PolyhedralMesh mesh;
    mesh.NumCell0Ds = 3;
    mesh.Cell0DsId = {0, 1, 2};
    mesh.Cell0DsCoordinates.resize(3, 3);

    mesh.Cell0DsCoordinates.col(0) = Vector3d(1.0, 0.0, 0.0);
    mesh.Cell0DsCoordinates.col(1) = Vector3d(0.0, 1.0, 0.0);
    mesh.Cell0DsCoordinates.col(2) = Vector3d(0.0, 0.0, 1.0);

    EXPECT_TRUE(Normalize(mesh));

    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d v = mesh.Cell0DsCoordinates.col(i);
        EXPECT_NEAR(v.norm(), 1.0, 1e-6);
    }
}

TEST(TestPolyhedron, TestNormalize3) {
    PolyhedralMesh mesh;
    mesh.NumCell0Ds = 3;
    mesh.Cell0DsId = {0, 1, 2};
    mesh.Cell0DsCoordinates.resize(3, 3);

    mesh.Cell0DsCoordinates.col(0) = Vector3d(2.0, 0.0, 0.0);
    mesh.Cell0DsCoordinates.col(1) = Vector3d(0.0, 0.0, -5.0);
    mesh.Cell0DsCoordinates.col(2) = Vector3d(1.0, 2.0, 2.0);

    EXPECT_TRUE(Normalize(mesh));

    for (int i = 0; i < 3; ++i) {
        Vector3d v = mesh.Cell0DsCoordinates.col(i);
        EXPECT_NEAR(v.norm(), 1.0, 1e-6);
    }
}




TEST(TestPolyhedron, TestCreateTriFace)
{
	PolyhedralMesh mesh;
	mesh.NumCell0Ds = 4;
	mesh.Cell0DsId.resize(mesh.NumCell0Ds);
	mesh.Cell0DsId = {0,1,2,3};
	mesh.Cell0DsCoordinates.resize(3, mesh.NumCell0Ds);
	mesh.Cell0DsCoordinates << 0.0, 1.0, 1.0, 0.0,
							   0.0, 0.0, 1.0, 1.0,
							   0.0, 0.0, 0.0, 0.0;

	mesh.NumCell1Ds = 6;
	mesh.Cell1DsId.resize(mesh.NumCell1Ds);
	mesh.Cell1DsId = {0,1,2,3,4,5};
	mesh.Cell1DsExtrema.resize(2, mesh.NumCell1Ds);
	mesh.Cell1DsExtrema << 0, 0, 0, 1, 1, 3,
						   1, 2, 3, 2, 3, 2;


	CreateTriFace(mesh, 3, 3, 2, 2);
	EXPECT_EQ(mesh.NumCell2Ds, 4);
}

TEST(TestPolyhedron, TestTriangulateClassII)
{
	PolyhedralMesh mesh;
	mesh.NumCell0Ds = 4;
	mesh.Cell0DsId.resize(mesh.NumCell0Ds);
	mesh.Cell0DsId = {0,1,2,3};
	mesh.Cell0DsCoordinates.resize(3, mesh.NumCell0Ds);
	mesh.Cell0DsCoordinates << 0.0, 1.0, 0.0, 0.0,
							   0.0, 0.0, 1.0, 0.0,
							   0.0, 0.0, 0.0, 1.0;

	mesh.NumCell1Ds = 6;
	mesh.Cell1DsId.resize(mesh.NumCell1Ds);
	mesh.Cell1DsId = {0,1,2,3,4,5};
	mesh.Cell1DsExtrema.resize(2, mesh.NumCell1Ds);
	mesh.Cell1DsExtrema << 0, 0, 0, 1, 1, 3,
						   1, 2, 3, 2, 3, 2;
	mesh.NumCell2Ds = 4;
	mesh.Cell2DsId.resize(mesh.NumCell2Ds);
	mesh.Cell2DsId = {0,1,2,3};
	mesh.Cell2DsVertices.resize(mesh.NumCell2Ds);
	mesh.Cell2DsVertices = {
		{0, 1, 2},
		{0, 1, 3},
		{0, 2, 3},
		{1, 2, 3}
	};
	mesh.Cell2DsEdges.resize(mesh.NumCell2Ds);
	mesh.Cell2DsEdges = {
		{0, 1, 3},
		{0, 1, 4},
		{1, 5, 2},
		{3, 5, 4}
	};
						   
						   
						   

	bool result = Triangulate(mesh, 3, 3, 1, 1);
	EXPECT_TRUE(result);
	EXPECT_EQ(mesh.NumCell1Ds, 36);
	EXPECT_EQ(mesh.NumCell0Ds, 14);	
}

TEST(TestPolyhedron, TestCentralize1) {
    PolyhedralMesh mesh;
    mesh.NumCell0Ds = 3;
    mesh.Cell0DsId = {0, 1, 2};
    mesh.Cell0DsCoordinates.resize(3, 3);

    // Tre punti non centrati
    mesh.Cell0DsCoordinates.col(0) = Vector3d(1.0, 0.0, 0.0);
    mesh.Cell0DsCoordinates.col(1) = Vector3d(2.0, 1.0, -1.0);
    mesh.Cell0DsCoordinates.col(2) = Vector3d(3.0, 2.0, 1.0);

    EXPECT_TRUE(Centralize(mesh));

    // Calcolo il nuovo baricentro
    Vector3d centroid = Vector3d::Zero();
    for (int i = 0; i < 3; ++i)
        centroid += mesh.Cell0DsCoordinates.col(i);

    centroid /= 3.0;

    // Il baricentro deve essere (0,0,0) con una tolleranza
    EXPECT_NEAR(centroid(0), 0.0, 1e-9);
    EXPECT_NEAR(centroid(1), 0.0, 1e-9);
    EXPECT_NEAR(centroid(2), 0.0, 1e-9);
}

TEST(TestPolyhedron, TestCentralize2) {
    PolyhedralMesh mesh;
    mesh.NumCell0Ds = 2;
    mesh.Cell0DsId = {0, 1};
    mesh.Cell0DsCoordinates.resize(3, 2);

    // Due punti simmetrici rispetto all'origine
    mesh.Cell0DsCoordinates.col(0) = Vector3d(-1.0, 0.0, 0.0);
    mesh.Cell0DsCoordinates.col(1) = Vector3d(1.0, 0.0, 0.0);

    EXPECT_TRUE(Centralize(mesh));

    // Dovrebbero essere spostati in -1 e +1 -> -1-0.5 = -1.5 e 1-0.5 = 0.5 -> quindi il centro è (0,0,0)
    Vector3d centroid = (mesh.Cell0DsCoordinates.col(0) + mesh.Cell0DsCoordinates.col(1)) / 2.0;
    EXPECT_NEAR(centroid.norm(), 0.0, 1e-9);
}

TEST(TestPolyhedron, TestTriangulateClassI)
{
	PolyhedralMesh mesh;
	mesh.NumCell0Ds = 4;
	mesh.Cell0DsId.resize(mesh.NumCell0Ds);
	mesh.Cell0DsId = {0,1,2,3};
	mesh.Cell0DsCoordinates.resize(3, mesh.NumCell0Ds);
	mesh.Cell0DsCoordinates << 1.0, 1.0, 0.0, 0.0,
							   1.0, 0.0, 1.0, 0.0,
							   1.0, 0.0, 0.0, 1.0;

	mesh.NumCell1Ds = 6;
	mesh.Cell1DsId.resize(mesh.NumCell1Ds);
	mesh.Cell1DsId = {0,1,2,3,4,5};
	mesh.Cell1DsExtrema.resize(2, mesh.NumCell1Ds);
	mesh.Cell1DsExtrema << 0, 0, 0, 1, 1, 3,
						   1, 2, 3, 2, 3, 2;
	mesh.NumCell2Ds = 4;
	mesh.Cell2DsId.resize(mesh.NumCell2Ds);
	mesh.Cell2DsId = {0, 1, 2, 3};
	mesh.Cell2DsVertices.resize(mesh.NumCell2Ds);
	mesh.Cell2DsVertices = {
		{0, 1, 2},
		{0, 1, 3},
		{0, 2, 3},
		{1, 2, 3}
	};
	mesh.Cell2DsEdges.resize(mesh.NumCell2Ds);
	mesh.Cell2DsEdges = {
		{0, 3, 1},
		{0, 4, 2},
		{1, 5, 2},
		{3, 5, 4}
	};
	bool result = Triangulate(mesh, 3, 3, 2, 0);
	int T = 2*2;
	EXPECT_TRUE(result);
	EXPECT_EQ(mesh.NumCell1Ds, 6*T);
	EXPECT_EQ(mesh.NumCell0Ds, 2*T + 2);
}

TEST(TestPolyhedron, TestGetorCreateEdge1)
{
	PolyhedralMesh mesh;
	mesh.NumCell1Ds = 0;
	unsigned int x = 0, y = 1, l = 0;
	unsigned int edgeId = GetorCreateEdge(mesh, x, y, l);
	EXPECT_EQ(edgeId, 0);
	EXPECT_EQ(mesh.NumCell1Ds, 1);
	EXPECT_EQ(mesh.Cell1DsExtrema(0, edgeId), x);
	EXPECT_EQ(mesh.Cell1DsExtrema(1, edgeId), y);
}

TEST(TestPolyhedron, TestGetorCreateEdge2)
{
	PolyhedralMesh mesh;
	mesh.NumCell1Ds = 1;
	mesh.Cell1DsId.push_back(0); 
	mesh.Cell1DsExtrema.resize(2, mesh.NumCell1Ds);
	mesh.Cell1DsExtrema << 0, 1;
	unsigned int x = 0, y = 1;
	unsigned int edgeId = GetorCreateEdge(mesh, x, y, 0);
	EXPECT_EQ(edgeId, 0); 
	EXPECT_EQ(mesh.NumCell1Ds, 1); 
}

TEST(TestPolyhedron, TestGetorCreatePoint1)
{
	PolyhedralMesh mesh;
	mesh.NumCell0Ds = 0;
    mesh.NumCell2Ds = 0;
	double x = 0.0, y = 1.0, z = 2.0;
	unsigned int l = 0;
	unsigned int pointId = GetorCreatePoint(mesh, x, y, z, l);
	EXPECT_EQ(pointId, 0);
	EXPECT_EQ(mesh.NumCell0Ds, 1);
	EXPECT_DOUBLE_EQ(mesh.Cell0DsCoordinates(0, pointId), x);
	EXPECT_DOUBLE_EQ(mesh.Cell0DsCoordinates(1, pointId), y);
	EXPECT_DOUBLE_EQ(mesh.Cell0DsCoordinates(2, pointId), z);
}

TEST(TestPolyhedron, TestGetorCreatePoint2)
{
	PolyhedralMesh mesh;
	mesh.NumCell0Ds = 1;
	mesh.NumCell1Ds = 1;
	mesh.Cell0DsId.push_back(0);
	mesh.Cell0DsCoordinates.resize(3, mesh.NumCell0Ds);
	mesh.Cell0DsCoordinates << 0.0,
							   1.0,
							   2.0;
	double x = 0.0, y = 1.0, z = 2.0;
	unsigned int l = 0;
	unsigned int pointId = GetorCreatePoint(mesh, x, y, z, l);
	EXPECT_EQ(pointId, 0); 
	EXPECT_EQ(mesh.NumCell0Ds, 1); 
	EXPECT_DOUBLE_EQ(mesh.Cell0DsCoordinates(0, pointId), x);
	EXPECT_DOUBLE_EQ(mesh.Cell0DsCoordinates(1, pointId), y);
	EXPECT_DOUBLE_EQ(mesh.Cell0DsCoordinates(2, pointId), z);
}

TEST(BuildAdjacencyTest, Grafo7Nodi) {
	std::vector<std::vector<unsigned int>> faces = {
		{0, 1}, {0, 2}, {1, 3}, {2, 3}, {2, 4}, {3, 5}, {4, 5}, {4, 6}, {5, 6}
	};
	
	unordered_map<unsigned int, unordered_set<unsigned int>> adjacency = buildAdjacencyList(faces);

	EXPECT_EQ(adjacency[0].size(), 2);
	EXPECT_EQ(adjacency[4].size(), 3);

	EXPECT_TRUE(adjacency[0].count(1));
	EXPECT_TRUE(adjacency[4].count(6));
}

TEST(BfsShortestPathTest, Grafo7Nodi) {
	std::vector<std::vector<unsigned int>> faces = {
		{0, 1}, {0, 2}, {1, 3}, {2, 3}, {2, 4}, {3, 5}, {4, 5}, {4, 6}, {5, 6}
	};
	
	unordered_map<unsigned int, unordered_set<unsigned int>> adjacency = buildAdjacencyList(faces);


	// Verifica alcune proprietà della lista di adiacenza
	EXPECT_EQ(adjacency[0].size(), 2);  // nodi adiacenti a 0: 1 e 2
	EXPECT_EQ(adjacency[4].size(), 3);  // nodi adiacenti a 4: 2,5,6

// Prepara le strutture richieste da bfs_shortest_path
	map<unsigned int, list<unsigned int>> Cell0DsMarker;
	map<unsigned int, list<unsigned int>> Cell1DsMarker;

// Inserisci qualche dato dummy se serve 
	Cell0DsMarker[0] = {10};
	Cell1DsMarker[1] = {20};

	unsigned int NumCell1Ds = 9;

	MatrixXi Cell1DsExtrema(2, NumCell1Ds);
	Cell1DsExtrema << 0, 0, 1, 2, 2, 3, 4, 4, 5,
			  1, 2, 3, 3, 4, 5, 5, 6, 6;
	

int n = 7;
unsigned int start = 0;
unsigned int end = 6;

// Chiama bfs_shortest_path
list<unsigned int> path = bfs_shortest_path(adjacency, start, end, n, Cell0DsMarker, Cell1DsMarker, NumCell1Ds, Cell1DsExtrema);

// Percorsi attesi (esempio)
vector<unsigned int> expected_path1 = {0, 2, 4, 6};


vector<unsigned int> actual_path(path.begin(), path.end());

// Controlla che il percorso trovato sia uno di quelli attesi
EXPECT_TRUE(actual_path == expected_path1);
	
}

TEST(BfsShortestPathTest, Grafo12Nodi_PercorsoPiuBreveUnico) {
	std::vector<std::vector<unsigned int>> faces = {
		{0, 1}, {1, 2}, {2, 3}, {3, 4},
		{4, 5}, {5, 6}, {6, 7}, {7, 8},
		{8, 9}, {9, 10}, {10, 11}, // percorso principale (lineare)
		{0, 5}, {1, 6}, {2, 7},    // archi alternativi più lunghi
		{3, 8}, {4, 9}             // ma non più brevi
	};

	unordered_map<unsigned int, unordered_set<unsigned int>> adjacency = buildAdjacencyList(faces);

	// Verifica proprietà della lista di adiacenza
	EXPECT_EQ(adjacency[0].size(), 2);   // 1, 5
	EXPECT_EQ(adjacency[5].size(), 3);   // 4, 6, 0
	EXPECT_EQ(adjacency[10].size(), 2);  // 9, 11

	// Marker dummy
	map<unsigned int, list<unsigned int>> Cell0DsMarker;
	map<unsigned int, list<unsigned int>> Cell1DsMarker;

	Cell0DsMarker[0] = {123};
	Cell1DsMarker[1] = {456};

	unsigned int NumCell1Ds = faces.size(); // 16 archi

	MatrixXi Cell1DsExtrema(2, NumCell1Ds);
	for (size_t i = 0; i < faces.size(); ++i) {
		Cell1DsExtrema(0, i) = faces[i][0];
		Cell1DsExtrema(1, i) = faces[i][1];
	}

	int n = 12;
	unsigned int start = 0;
	unsigned int end = 11;

	// Chiama bfs_shortest_path
	list<unsigned int> path = bfs_shortest_path(
		adjacency, start, end, n,
		Cell0DsMarker, Cell1DsMarker,
		NumCell1Ds, Cell1DsExtrema
	);

	// Percorso più breve unico e verificabile
	vector<unsigned int> expected_path = {0, 5, 4, 9, 10, 11};

	vector<unsigned int> actual_path(path.begin(), path.end());

	EXPECT_EQ(actual_path, expected_path);
}

