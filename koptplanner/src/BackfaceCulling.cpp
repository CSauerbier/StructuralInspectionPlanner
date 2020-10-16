#include "Culling/BackfaceCulling.h"

std::unordered_set<tri_t*> BackfaceCulling::getFrontFacets(Eigen::Matrix<float, 5, 1> vp, std::unordered_set<tri_t*> tri_checked)
{
    std::unordered_set<tri_t*> return_set;

    //Convert headings to vector. Pitch direction needs to be inverted (up should be positive)
    Eigen::Vector3f vp_pose;
    vp_pose << cos(vp[3])*cos(-vp[4]), sin(vp[3])*cos(-vp[4]), sin(-vp[4]);

    for(auto tri: tri_checked)
    {
        //Cross product of (Vertex2-Vertex1) and (Vertex3-Vertex2) yields surface normal
        Eigen::Vector3f normal;
        normal = (tri->x2 - tri->x1).cross(tri->x3 - tri->x2);

        //if normal vector points in opposite direction of vp vector, front face is towards vp
        if(vp_pose.dot(normal) < 0)
        {
            return_set.insert(tri);
        }
    }

    return return_set;
}