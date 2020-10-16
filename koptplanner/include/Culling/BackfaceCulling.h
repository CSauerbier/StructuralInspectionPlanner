#pragma once

#include "koptplanner/TriangleObject.h"
#include "koptplanner/ptpPlanner.hpp"
#include <eigen3/Eigen/Dense>
#include <unordered_set>
#include "tf/tf.h"
#include <math.h>


namespace BackfaceCulling
{
    //Iterates over input set and returns only those entries that have their normals pointing towards the vp 
    std::unordered_set<tri_t*> getFrontFacets(Eigen::Matrix<float, 5, 1> vp, std::unordered_set<tri_t*> tri_checked);
}