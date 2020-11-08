#include "Culling/HiddenSurfaceRemoval.h"

TriExtrusion::TriExtrusion(tri_t* tri)
{
    this->tri_orig = tri;

    this->tri_plane = Eigen::Hyperplane<float, 3>::Through(tri->x1, tri->x2, tri->x3);


    this->boundary_planes[0] = Eigen::Hyperplane<float, 3>::Through(tri->x1, tri->x2, tri->x2 + this->tri_plane.normal());
    this->boundary_planes[1] = Eigen::Hyperplane<float, 3>::Through(tri->x2, tri->x3, tri->x3 + this->tri_plane.normal());
    this->boundary_planes[2] = Eigen::Hyperplane<float, 3>::Through(tri->x3, tri->x1, tri->x1 + this->tri_plane.normal());

    this->normal = (tri->x2-tri->x1).cross(tri->x3-tri->x2);
}

//Checks if a point lies within a body defined by the lines connecting the triangle vertices, extruded in both directions along the surface normal
bool TriExtrusion::isWithinExtrusion(Eigen::Vector3f point)
{
    for(auto plane: this->boundary_planes)
    {
        if(plane.signedDistance(point) > 0) return false;
    }
    return true;
}

//Returns absolute distance of the point to the hyperplane
float TriExtrusion::distanceToPlane(Eigen::Vector3f point)
{
    return this->tri_plane.absDistance(point);
}

//Links mesh_fine entries by assigning them to the geometrically closest entry of mesh_coarse_all
//Returns a set containing those elements of mesh_fine that are not assigned to a facet of mesh_coarse_invisible
std::unordered_set<tri_t*> HiddenSurfaceRemoval::removeHiddenSurfaces(std::unordered_set<tri_t*> mesh_coarse_all, std::unordered_set<tri_t*> mesh_coarse_invisible, std::unordered_set<tri_t*> mesh_fine)
{
    std::unordered_set<tri_t*> mesh_coarse_visible = subtractSet(mesh_coarse_all, mesh_coarse_invisible);
    
    std::unordered_set<tri_t*> possibly_visible_surfaces;

    std::unordered_set<Eigen::Vector3f*> visible_vertices = getVisibleVertices(mesh_coarse_visible);

    std::unordered_set<tri_t*> tri_visible_or_adjacent = findVisibleAndAdjacent(mesh_coarse_visible, visible_vertices);

    std::unordered_map<tri_t*, tri_t*> linked_meshes = linkMeshElements(mesh_coarse_all, mesh_fine);

    auto end = tri_visible_or_adjacent.end();
    for(auto map_entry: linked_meshes)
    {
        if(tri_visible_or_adjacent.find(map_entry.second) != end)
        {
            possibly_visible_surfaces.insert(map_entry.first);
        }
    }

    return possibly_visible_surfaces;
}

//std::vector wrapper for std::set variant of removeHiddenSurfaces
std::vector<tri_t*> HiddenSurfaceRemoval::removeHiddenSurfaces(std::vector<tri_t*> coarse_all, std::vector<tri_t*> coarse_visible, std::vector<tri_t*> fine)
{
    std::unordered_set<tri_t*> coarse_all_map (coarse_all.begin(), coarse_all.end());
    std::unordered_set<tri_t*> coarse_visible_map (coarse_visible.begin(), coarse_visible.end());
    std::unordered_set<tri_t*> fine_map (fine.begin(), fine.end());

    std::unordered_set<tri_t*> set_result = HiddenSurfaceRemoval::removeHiddenSurfaces(coarse_all_map, coarse_visible_map, fine_map);
    std::vector<tri_t*> vector_result (set_result.begin(), set_result.end());
    return vector_result;
}

//Returns the components of full_mesh, that comprise at least one visibile vertex
std::unordered_set<tri_t*> findVisibleAndAdjacent(std::unordered_set<tri_t*> full_mesh, std::unordered_set<Eigen::Vector3f*> visible_vertices)
{
    auto end = visible_vertices.end();
    std::unordered_set<tri_t*> return_set;
    for(auto tri: full_mesh)
    {
        
        if( (visible_vertices.find(&(tri->x1)) != end) ||
            (visible_vertices.find(&(tri->x2)) != end) ||
            (visible_vertices.find(&(tri->x3)) != end))
        {
            return_set.insert(tri);
        }
    }
    return return_set;
}

std::unordered_set<Eigen::Vector3f*> getVisibleVertices(std::unordered_set<tri_t*> mesh_visible)
{
    std::unordered_set<Eigen::Vector3f*> return_set;
    for(auto tri: mesh_visible)
    {
        return_set.insert(&(tri->x1));
        return_set.insert(&(tri->x2));
        return_set.insert(&(tri->x3));
    }
    return return_set;
}

//Maps all mesh_fine mesh facets to the closest coarse mesh facet respectively
//Returns map that holds mesh_fine mesh facets as keys and coarse mesh entries as values
std::unordered_map<tri_t*, tri_t*> linkMeshElements(std::unordered_set<tri_t*> coarse_mesh, std::unordered_set<tri_t*> fine_mesh)
{
    std::unordered_map<tri_t*, tri_t*> return_map;

    std::unordered_map<tri_t*, Eigen::Vector3f*> map_center_points_fine;
    for(auto tri_f: fine_mesh)
    {
        
        Eigen::Vector3f* center = new Eigen::Vector3f;
        *center = (tri_f->x1 + tri_f->x2 + tri_f->x3)/3;
        map_center_points_fine[tri_f] = center;
    }

    std::unordered_map<tri_t*, TriExtrusion*> tri_extrusions;
    for(auto tri_c: coarse_mesh)
    {
        TriExtrusion* tri_ex = new TriExtrusion(tri_c);
        tri_extrusions[tri_c] = tri_ex;
    }

    std::vector<tri_t*> not_within_extrusion;
    for(auto tri_f: fine_mesh)
    {
        Eigen::Vector3f center = (tri_f->x1 + tri_f->x2 + tri_f->x3)/3;
        float shortest_dist = MAXFLOAT;
        tri_t* closest_tri = NULL;

        for(auto plane: tri_extrusions)
        {
            //If surface normals are not facing into opposite directions
            Eigen::Vector3f normal_f = (tri_f->x2-tri_f->x1).cross(tri_f->x3-tri_f->x2);
            if(normal_f.dot(plane.second->normal) > 0)
            {
                //If the fine mesh center point lies within the extrusion of the triangle, the distance to the hyperplane is used
                if(plane.second->isWithinExtrusion(center))
                {
                    float distance = plane.second->distanceToPlane(center);
                    if(distance < shortest_dist)
                    {
                        shortest_dist = distance;
                        closest_tri = plane.first;
                    }
                }
            }
        }
        //If the center point lies outside the extrusion, the fine_mesh facet is assigned to one of the coarse_mesh facet that comprises the closest vertex
        if(closest_tri == NULL)
        {
            not_within_extrusion.push_back(tri_f);
            for(auto tri_c: coarse_mesh)
            {
                float distance = (tri_c->x1 - center).norm();
                if(distance < shortest_dist)
                {
                    shortest_dist = distance;
                    closest_tri = tri_c;
                }
                distance = (tri_c->x2 - center).norm();
                if(distance < shortest_dist)
                {
                    shortest_dist = distance;
                    closest_tri = tri_c;
                }
                distance = (tri_c->x3 - center).norm();
                if(distance < shortest_dist)
                {
                    shortest_dist = distance;
                    closest_tri = tri_c;
                }
            }
        }
        return_map[tri_f] = closest_tri;
    }

    for(auto plane: tri_extrusions)
    {
        delete plane.second;
    }

    return return_map;
}

//Performs a set subtraction by only returning the elements of full_set that are not contained in part_set
std::unordered_set<tri_t*> subtractSet(std::unordered_set<tri_t*>full_set, std::unordered_set<tri_t*>part_set)
{
    std::unordered_set<tri_t*> return_set;
    auto end = part_set.end();
    for(auto f_entry: full_set)
    {
        if(part_set.find(f_entry) == end)
        {
            return_set.insert(f_entry);
        }
    }

    return return_set;
}