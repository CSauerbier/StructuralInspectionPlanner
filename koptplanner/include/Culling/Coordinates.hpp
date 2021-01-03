#pragma once
#include <array>


struct CartesianCoordinates
{
    float x, y, z;

    CartesianCoordinates(float x, float y, float z);
};


struct TriangleVertices
{
    std::array<CartesianCoordinates*, 3> vertices;

    TriangleVertices(CartesianCoordinates&, CartesianCoordinates&, CartesianCoordinates&);
    ~TriangleVertices();
};

