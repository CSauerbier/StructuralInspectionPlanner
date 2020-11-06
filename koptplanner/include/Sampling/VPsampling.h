#pragma once

#include "ros/ros.h"
#include "koptplanner/TriangleObject.h"
#include "koptplanner/ptpPlanner.hpp"
#include "eigen3/Eigen/Dense"
#include <array>
#include <numeric>
#include <random>
#include <time.h>

class RandomSampling
{
private:
    /**
     * Returns a random number from within the specified interval
     * \param low lower bound of interval
     * \param high higher bound of interval
     * \return Value from within the specified interval        
    */
    static float randNum(float low, float high);

    /**
     * \return Random number from within the interval (0,1)
    */
    static float randNum();

    RandomSampling();

public:
    /**
     * Generate a random point from the space within the maximum/minimum distance constraints and the incidence angle constraint
     * \param tri Triangle object to sample the view point for
     * \return Randomly sampled position within the boundaries
    */
    static StateVector getVP(tri_t* tri, bool debug);//TO-DO: REmove debug flag
};
