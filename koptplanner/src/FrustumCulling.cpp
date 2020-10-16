#include "Culling/FrustumCulling.h"

std::unordered_set<tri_t*> FrustumCulling::getFacetsWithinFrustum(std::unordered_set<tri_t*> inputTriangles, Eigen::Matrix<float, 5, 1> vp, bool include_boundary_facets)
{
    float cam_angle_vertical, cam_angle_horizontal;
    ros::param::get("~/camera/vertical", cam_angle_vertical);
    ros::param::get("~/camera/horizontal", cam_angle_horizontal);

    std::unordered_set<tri_t*> return_set;

    std::set<TriTransformed*> tri_tr_container = convertToVPCoSys(inputTriangles, vp);

    for(auto tri_tr: tri_tr_container)
    {
        bool is_partly_outside = false;
        bool is_partly_within = false;

        float angle_v, angle_h;
        for(auto vertex: tri_tr->vertices_tf)
        {

            angle_h = std::atan(vertex.getY()/vertex.getX());
            angle_v = std::atan(vertex.getZ()/vertex.getX());

            if( (std::abs(angle_h) > cam_angle_horizontal/2) ||
                (std::abs(angle_v) > cam_angle_vertical/2))
            {
                is_partly_outside = true;
            }
            else
            {
                is_partly_within = true;
            }
        }
        //If completely within
        if(!is_partly_outside)
        {
            return_set.insert(tri_tr->tri_original);
        }
        else if(is_partly_within && include_boundary_facets)
        {
            return_set.insert(tri_tr->tri_original);
        }        
    }
    return return_set;
}

std::set<TriTransformed*> convertToVPCoSys(std::unordered_set<tri_t*> inputTriangles, Eigen::Matrix<float, 5, 1> vp)
{
    std::set<TriTransformed*> return_set;

    //Calculate transformation matrix to VP frame
    tf2::Vector3 origin;
    origin.setX(vp[0]);
    origin.setY(vp[1]);
    origin.setZ(vp[2]);

    tf2::Quaternion rotation;
    rotation.setRPY(0, vp[4], vp[3]);

    tf2::Transform tf_inv;
    tf_inv.setOrigin(origin);
    tf_inv.setRotation(rotation);
    tf2::Transform tf = tf_inv.inverse();

    //Iterate over all input triangles and create data structure of facets in spherical coordinates 
    std::set<std::array<tf2::Vector3, 3>> tri_sp_container;
    for(auto it: inputTriangles)
    {
        std::array<tf2::Vector3, 3UL> vert_tf;
        std::array<Eigen::Vector3f,3> vertices_eig {it->x1, it->x2, it->x3};
        
        for(int i=0; i<vertices_eig.size(); i++)
        {
            vert_tf[i] = tf * tf2::Vector3(vertices_eig[i][0], vertices_eig[i][1], vertices_eig[i][2]);
        }

        TriTransformed* tri_tr = new TriTransformed(it, vert_tf);
        return_set.insert(tri_tr);
    }
    return return_set;
}

TriTransformed::TriTransformed(tri_t* tri_orig, std::array<tf2::Vector3, 3> vert_tf)
{
    this->tri_original = tri_orig;
    this->vertices_tf = vert_tf;
}