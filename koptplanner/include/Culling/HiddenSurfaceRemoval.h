#pragma once

#include "koptplanner/TriangleObject.h"
#include "koptplanner/ptpPlanner.hpp"
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <array>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>

//Holds hyperplanes of the triangle itself and planes defined by the boundary lines and the hyperplane normal
class TriExtrusion
{
    tri_t* tri_orig;
    Eigen::Hyperplane<float, 3> tri_plane;
    std::array<Eigen::Hyperplane<float, 3>, 3> boundary_planes;

public:
    Eigen::Vector3f normal;
    TriExtrusion(tri_t*);
    bool isWithinExtrusion(Eigen::Vector3f);
    float distanceToPlane(Eigen::Vector3f);
};


namespace HiddenSurfaceRemoval
{
    std::unordered_set<tri_t*> removeHiddenSurfaces(std::unordered_set<tri_t*> coarse_all, std::unordered_set<tri_t*> coarse_visible, std::unordered_set<tri_t*> fine);
    std::vector<tri_t*> removeHiddenSurfaces(std::vector<tri_t*> coarse_all, std::vector<tri_t*> coarse_visible, std::vector<tri_t*> fine);
}

std::unordered_set<tri_t*> findVisibleAndAdjacent(std::unordered_set<tri_t*> full_mesh, std::unordered_set<Eigen::Vector3f*> visible_vertices);
std::unordered_set<Eigen::Vector3f*> getVisibleVertices(std::unordered_set<tri_t*> mesh_visible);
std::unordered_map<tri_t*, tri_t*> linkMeshElements(std::unordered_set<tri_t*> coarse_mesh, std::unordered_set<tri_t*> fine_mesh);
std::unordered_set<tri_t*> subtractSet(std::unordered_set<tri_t*>full_set, std::unordered_set<tri_t*>part_set);