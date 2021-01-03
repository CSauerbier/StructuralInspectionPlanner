#include "Culling/Coordinates.hpp"


CartesianCoordinates::CartesianCoordinates(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}


TriangleVertices::TriangleVertices(CartesianCoordinates &p0, CartesianCoordinates &p1, CartesianCoordinates &p2)
{
    this->vertices[0] = new CartesianCoordinates(p0.x, p0.y, p0.z);
    this->vertices[1] = new CartesianCoordinates(p1.x, p1.y, p1.z);
    this->vertices[2] = new CartesianCoordinates(p2.x, p2.y, p2.z);
}

TriangleVertices::~TriangleVertices()
{
    for(auto v: this->vertices)
    {
        delete v;
        v = NULL;
    }
}