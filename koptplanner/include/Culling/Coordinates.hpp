#pragma once
#include <array>


/**
 * Data structure to hold cartesian coordinates x,y,z
 */
struct CartesianCoordinates
{
    float x, y, z;

    CartesianCoordinates(float x, float y, float z);
};

/**
 * Data structure to store a 3D triangle composed of three points in cartesian coordinates
 */
struct TriangleVertices
{
    std::array<CartesianCoordinates*, 3> vertices;

    /**
     * Constructs a triangle object from three points. Contents of the points are copied into new variables.
     */
    TriangleVertices(CartesianCoordinates &p0, CartesianCoordinates &p1, CartesianCoordinates&p2);

    /**
     * Destructs the defining points
     */
    ~TriangleVertices();
};

