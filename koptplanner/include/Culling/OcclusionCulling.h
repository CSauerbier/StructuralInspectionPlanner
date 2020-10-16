#pragma once

#include "koptplanner/TriangleObject.h"
#include "koptplanner/ptpPlanner.hpp"
#include <vector>
#include <unordered_set>
#include <array>
#include <map>
#include <eigen3/Eigen/Dense>
#include <boost/geometry.hpp>
#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>

namespace bg = boost::geometry;

//TO-DO: Consider using float instead of double throughout

//Representation of triangle in a spherical coordinate system as seen from the respective VP
struct TriSpherical
{
    tri_t* tri_orig;
    std::array<bg::model::point<float, 3, bg::cs::spherical<bg::degree> >, 3> vertices;
    float mean_distance_to_vp;
};

struct PointSpherical
{
    bool is_occluded = false;
    std::unordered_set<tri_t*> tris_attached;
    bg::model::point<float, 3, bg::cs::spherical<bg::degree> > vertex;
};

bg::model::point<float, 2, bg::cs::cartesian> dropZCoordinate(bg::model::point<float, 3, bg::cs::spherical<bg::degree>>);
void setFullyCovered();



//Check if this class takes up too much memory
class RasterElement
{
private:
    //Is only set if the grid element ist fully covered and the closest element. 
    //Is set to false if covered by another triangle, that may not fully cover the cell
    bool is_completely_covered = false;
    TriSpherical* completely_covered_by;
    
public:
    std::unordered_set<TriSpherical*> tri_contained;

    bool isCompletelyCovered();
    void setCompletelyCovered(bool);
    TriSpherical* getCoveringTriangle();
    void setCoveringTriangle(TriSpherical*);
};


//TO-DO_old: Consider using class, especially for fillRasterElement
namespace OcclusionCulling
{
    std::unordered_set<tri_t*> getUnoccludedFacets(Eigen::Matrix<float, 5, 1> vp, std::unordered_set<tri_t*> tri_checked, std::unordered_set<tri_t*>tri_considered);
    std::unordered_set<tri_t*> getUnoccludedFacets_w_zBuffer(Eigen::Matrix<float, 5, 1> vp, std::unordered_set<tri_t*> tri_checked, std::unordered_set<tri_t*>tri_considered);
    void fillRasterElement(RasterElement &ras_el, TriSpherical *tri_sp, std::unordered_set<tri_t*> &tri_spherical_to_return);
    bool isPointInsideTriangle();

    //Checks if point in first argument lies within triangle of second argument
    bool geometryIsWithin(bg::model::point<float, 2, bg::cs::cartesian> point_under_test, TriSpherical* tri_sp);
    //Checks if triangle in first argument lies within triangle of second argument
    char geometryIsWithin(TriSpherical* tri_sp_pot_ocludee, TriSpherical* tri_sp_occluder);

    bg::model::point<float, 3, bg::cs::spherical<bg::degree> > convertToSphericalInVPSys(Eigen::Vector3f point, tf2::Transform tf);
    std::array<bg::model::point<float, 3, bg::cs::spherical<bg::degree> >, 3> convertToSphericalInVPSys(tri_t* tri, tf2::Transform tf);

    enum occlusion_result
    {
        OUTSIDE = 0,
        INTERSECTING,
        WITHIN,
    };
}
