#include "Culling/DistanceCulling.h"

std::unordered_set<tri_t*> DistanceCulling::getFacetsWithinDistance(std::unordered_set<tri_t*> inputTriangles,
                                                                    Eigen::Matrix<float, 5, 1> vp)
{
    std::unordered_set<tri_t*> return_set;
    Eigen::Vector3f vp_pos(vp[0], vp[1], vp[2]);
    float max_dist, min_dist;
    ros::param::get("~/camera/min_dist", min_dist);
    ros::param::get("~/camera/max_dist", max_dist);

    for(auto tri: inputTriangles)
    {
        std::array<Eigen::Vector3f, 3> vertices {tri->x1, tri->x2, tri->x3};
        bool within = true;
        for(auto vertex: vertices)
        {
            float dist = (vertex-vp_pos).norm();
            if(dist>max_dist || dist<min_dist)
            {
                within = false;
                break;
            }
        }
        if(within)
        {
            return_set.insert(tri);
        }
    }

    return return_set;
}