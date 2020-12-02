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

#define THREADS_PER_BLOCK 1024

//TO-DO: Documentation

namespace bg = boost::geometry;

int *within_triangle_query_gpu_driver(int num, int threads ,
    float p_x,
    float p_y,
    float *v0_x, 
    float *v0_y,
    float *v1_x, 
    float *v1_y, 
    float *v2_x, 
    float *v2_y, 
    bool *output,
    bool use_gpu);

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

    /**
     * Checks whether the input points_checked map contents is covered by any of the tri_considered triangles.
     * If so, the correspoinding member of PointSpherical is set.
     * \param points_checked Point map, each entry is checked for occlusion and updated accordingly
     * \param tri_considered Potential occluders
     * */
    void occlusionCheck_usingBoost(std::map<std::tuple<float, float, float>, PointSpherical *> &points_checked, 
                            std::unordered_set<TriSpherical *> &tri_considered);

    /**
     * Checks whether the input points_checked map contents is covered by any of the tri_considered triangles.
     * If so, the correspoinding member of PointSpherical is set.
     * Wrapper to convert the input data structures to c arrays that can be passed to the CUDA API.
     * \param points_checked Point map, each entry is checked for occlusion and updated accordingly
     * \param tri_considered Potential occluders
     * */
    void occlusionCheck_GPU(std::map<std::tuple<float, float, float>, PointSpherical *> &points_checked, 
                            std::unordered_set<TriSpherical *> &tri_considered);
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
