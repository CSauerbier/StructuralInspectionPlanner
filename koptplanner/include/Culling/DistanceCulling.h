#pragma once

#include "koptplanner/TriangleObject.h"
#include "koptplanner/ptpPlanner.hpp"
#include <unordered_set>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "ros/ros.h"

namespace DistanceCulling
{
    /**
     * Culls elements from the input array that do no lie within the distance band specified by in parameters.yaml
     * \param input_triangles Set of triangles to be checked for whether or not they fulfill the distance constraint
     * \param vp View point coordinates and heading from which the input_triangles are viewed
     * \returns Set of triangles that fulfill the distance constraint
     * */
    std::unordered_set<tri_t*> getFacetsWithinDistance(std::unordered_set<tri_t*> input_triangles, Eigen::Matrix<float, 5, 1> vp);
}