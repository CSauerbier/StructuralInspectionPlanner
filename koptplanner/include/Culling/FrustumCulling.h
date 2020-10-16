#pragma once

#include "koptplanner/TriangleObject.h"
#include "koptplanner/ptpPlanner.hpp"
#include <unordered_set>
#include <array>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <math.h>

struct TriTransformed
{
    tri_t* tri_original;
    std::array<tf2::Vector3, 3> vertices_tf;

    TriTransformed(tri_t* tri_original, std::array<tf2::Vector3, 3> vertices_tf);
    TriTransformed();
};

namespace FrustumCulling
{
    std::unordered_set<tri_t*> getFacetsWithinFrustum(std::unordered_set<tri_t*> inputTriangles, Eigen::Matrix<float, 5, 1> vp, bool include_boundary_facets);
}

std::set<TriTransformed*> convertToVPCoSys(std::unordered_set<tri_t*> inputTriangles, Eigen::Matrix<float, 5, 1> vp);