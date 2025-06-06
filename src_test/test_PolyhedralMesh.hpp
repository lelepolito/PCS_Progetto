#pragma once

#include <iostream>
#include <vector>
#include <gtest/gtest.h>

#include "Utils.hpp"
#include "Eigen/Eigen"
#include "struttura.hpp"

using namespace Eigen;
using namespace std;
using namespace PolygonalLibrary;

TEST(TestPolyhedron, TestCentralize1)
{	
    PolyhedralMesh mesh;
	int n=6;
	mesh.NumCell0Ds = n;
	MatrixXd matrice(3,n);
	matrice << 0.593, -0.711,  0.322, -0.112,  0.001,  0.689,
               0.463,  0.245, -0.832, -0.408,  0.863, -0.655,
               0.659,  0.658,  0.451,  0.906, -0.505,  0.316;
	mesh.Cell0DsCoordinates = matrice.eval();
	PolyhedralMesh meshNorm;
	meshNorm.NumCell0Ds = n;
	meshNorm.Cell0DsCoordinates = mesh.Cell0DsCoordinates;
    Normalize(mesh);
	bool test = mesh.Cell0DsCoordinates.isApprox(meshNorm.Cell0DsCoordinates,1e-5);
	EXPECT_TRUE(test);
}

TEST(TestPolyhedron, TestCentralize2)
{	
    PolyhedralMesh mesh;
	int n=6;
	mesh.NumCell0Ds = n;
	MatrixXd matrice(3,n);
	matrice <<
    0.593 * 4.2,  -0.711 * 9.1,   0.322 * 2.8,  -0.112 * 6.7,   0.001 * 1.3,   0.689 * 7.5,
    0.463 * 4.2,   0.245 * 9.1,  -0.832 * 2.8,  -0.408 * 6.7,   0.863 * 1.3,  -0.655 * 7.5,
    0.659 * 4.2,   0.658 * 9.1,   0.451 * 2.8,   0.906 * 6.7,  -0.505 * 1.3,   0.316 * 7.5;
	mesh.Cell0DsCoordinates = matrice.eval();
    Normalize(mesh);
	PolyhedralMesh meshNorm;
	meshNorm.NumCell0Ds = n;
	MatrixXd matrice2(3,n);
	matrice2 <<  
    0.593, -0.711,  0.322, -0.112,  0.001,  0.689,
    0.463,  0.245, -0.832, -0.408,  0.863, -0.655,
    0.659,  0.658,  0.451,  0.906, -0.505,  0.316;
	meshNorm.Cell0DsCoordinates = matrice2.eval();
	bool test = mesh.Cell0DsCoordinates.isApprox(meshNorm.Cell0DsCoordinates,1e-5);
	EXPECT_TRUE(test);
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
	vector<vector<unsigned int>> expectedVertices = {
				  {0, 1, 2},
				  {0, 1, 3},
				  {0, 2, 3},
				  {1, 2, 3}};
	EXPECT_EQ(mesh.NumCell2Ds, 4);
	/*for (unsigned int i = 0; i < mesh.NumCell2Ds-1; ++i) {
		EXPECT_EQ(mesh.Cell2DsVertices[i], expectedVertices[i]);
	}*/
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

TEST(TestPolyhedron, TestTriangulateClassI)
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
	mesh.Cell0DsId.push_back(0);
	mesh.Cell0DsCoordinates.resize(3, mesh.NumCell0Ds);
	mesh.Cell0DsCoordinates << 0.0, 1.0, 2.0;
	double x = 0.0, y = 1.0, z = 2.0;
	unsigned int l = 0;
	unsigned int pointId = GetorCreatePoint(mesh, x, y, z, l);
	EXPECT_EQ(pointId, 0); 
	EXPECT_EQ(mesh.NumCell0Ds, 1); 
	EXPECT_DOUBLE_EQ(mesh.Cell0DsCoordinates(0, pointId), x);
	EXPECT_DOUBLE_EQ(mesh.Cell0DsCoordinates(1, pointId), y);
	EXPECT_DOUBLE_EQ(mesh.Cell0DsCoordinates(2, pointId), z);
}