#pragma once

#include "ros/ros.h"
#include "koptplanner/TriangleObject.h"
#include "koptplanner/ptpPlanner.hpp"
#include "eigen3/Eigen/Dense"
#include <array>
#include <numeric>
#include <random>
#include <time.h>
#include <cmath>


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
    static StateVector getVP(tri_t* tri);
};


class EquidistantPointsOnSphere
{
private:
    std::vector<StateVector*> view_points;
    std::vector<Eigen::Vector3f*> points;
    unsigned int counter;

    void samplePositionsUnitSphere(unsigned int n_target);
    void convertToStateVector(std::vector<double> center, float radius);
public:
    /**
     * Creates object and triggers computation of equidistantly distributed points on a view sphere.
     * Algorithm for equidistant sampling according to Deserno 2004
     * \param center Center of the view sphere to be created
     * \param radius Radius of the view sphere to be created
     * \param n_target Target number of points to be generated. Actual number of points might vary slightly
     * */
    EquidistantPointsOnSphere(std::vector<double> center, float radius, unsigned int n_target);
    
    /**
     * Destructor, deletes all member variables that require deletion
     * */
    ~EquidistantPointsOnSphere();

    /**
     * \returns Returns a view point sample and increments a counter, so that the next call of the function returns the next entry.
     * If the last entry is reached, this entry is always returned in successive calls.
     * */
    StateVector* getVP();

    /**
     * \returns Number of samples generated. Number might differ from n_target passed to constructor
     * */
    size_t numberOfPointsGenerated();
};
