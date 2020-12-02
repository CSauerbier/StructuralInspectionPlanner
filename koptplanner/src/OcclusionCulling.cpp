
#include "Culling/OcclusionCulling.h"

typedef int vert_ind;
typedef int horiz_ind;


std::unordered_set<tri_t*> OcclusionCulling::getUnoccludedFacets(Eigen::Matrix<float, 5, 1> vp,
                                                                 std::unordered_set<tri_t*> tri_checked, 
                                                                 std::unordered_set<tri_t*>tri_considered)
{
    std::unordered_set<tri_t*> tri_to_return;

    //Calculate transformation matrix to VP frame
    tf2::Vector3 origin;
    origin.setX(vp[0]);
    origin.setY(vp[1]);
    origin.setZ(vp[2]);

    tf2::Quaternion rotation;
    rotation.setRPY(0, -vp[4], vp[3]);

    tf2::Transform tf_inv;
    tf_inv.setOrigin(origin);
    tf_inv.setRotation(rotation);
    tf2::Transform tf = tf_inv.inverse();

    //Iterate over all input triangles and create data structure of facets in spherical coordinates 
    std::unordered_set<TriSpherical*> tri_sp_container;
    for(auto it = tri_considered.begin(); it != tri_considered.end(); it++)
    {
        std::array<boost::geometry::model::point<float, 3UL, boost::geometry::cs::spherical<boost::geometry::degree>>, 3UL> p_spherical;
        p_spherical = OcclusionCulling::convertToSphericalInVPSys(*it, tf);

        //Create Object to store spherical coordinates and reference to original object
        TriSpherical* tri_sp_temp = new TriSpherical;
        tri_sp_temp->tri_orig = *it;
        tri_sp_temp->vertices = p_spherical;
        float sum = 0;
        for(auto it2 = p_spherical.begin(); it2 != p_spherical.end(); it2++)
        {
            sum += (*it2).get<2>();
        }
        tri_sp_temp->mean_distance_to_vp = sum/p_spherical.size();

        tri_sp_container.insert(tri_sp_temp);
    }

    //Create set to hold points with their respective triangles
    //Use Vector3f as key? Might create duplicates because of handling objects not references
    std::map<std::tuple<float, float, float>, PointSpherical*> points_sp;
    for(auto tri: tri_checked)
    {
        Eigen::Vector3f point;
        for(int i=0; i<3; i++)
        {
            switch(i)
            {
            case 0:
                point = tri->x1;
                break;
            case 1:
                point = tri->x2;
                break;
            case 2:
                point = tri->x3;
                break;
            }
            
            auto tuple = std::make_tuple(point[0],point[1], point[2]);
            if(points_sp.find(tuple) == points_sp.end())
            {
                //Initialize element
                PointSpherical* p = new PointSpherical;
                p->vertex = OcclusionCulling::convertToSphericalInVPSys(point, tf);
                points_sp[tuple] = p;
            }
            points_sp[tuple]->tris_attached.insert(tri);
        } 
    }

    //occlusionCheck_usingBoost(points_sp, tri_sp_container);
    occlusionCheck_GPU(points_sp, tri_sp_container);
    
    tri_to_return = tri_checked;

    for(auto point: points_sp)
    {
        if(point.second->is_occluded)
        {
            for(auto tri: point.second->tris_attached)
            {
                tri_to_return.erase(tri);
            }
        }
    }

    //Clean up
    for(auto point: points_sp)
    {
        delete point.second;
    }
    points_sp.clear();

    for(auto tri: tri_sp_container)
    {
        delete tri;
    }
    tri_sp_container.clear();

    return tri_to_return;
}

std::unordered_set<tri_t*> OcclusionCulling::getUnoccludedFacets_w_zBuffer(Eigen::Matrix<float, 5, 1> vp, std::unordered_set<tri_t*> tri_checked, std::unordered_set<tri_t*>tri_considered)
{

    std::unordered_set<tri_t*> tri_to_return;

    float cam_angle_vertical, cam_angle_horizontal;
    //TO-DO_old: ROS-Parameter
    const int z_buffer_res = 1;
    std::map<std::pair<horiz_ind,vert_ind>, RasterElement> z_buffer;   //<horizontal, vertical>

    ros::param::get("~/camera/vertical", cam_angle_vertical);
    ros::param::get("~/camera/horizontal", cam_angle_horizontal);
    cam_angle_vertical = cam_angle_vertical*180/M_PI;
    cam_angle_horizontal = cam_angle_horizontal*180/M_PI;

    float angular_increment_h = cam_angle_horizontal/z_buffer_res;
    float angular_increment_v = cam_angle_vertical/z_buffer_res;

    //Calculate transformation matrix to VP frame
    tf2::Vector3 origin;
    origin.setX(vp[0]);
    origin.setY(vp[1]);
    origin.setZ(vp[2]);

    tf2::Quaternion rotation;
    rotation.setRPY(0, -vp[4], vp[3]);

    tf2::Transform tf_inv;
    tf_inv.setOrigin(origin);
    tf_inv.setRotation(rotation);
    tf2::Transform tf = tf_inv.inverse();


    //Iterate over all input triangles and order them according to their distance to the camera
    std::map<float, TriSpherical*> ordered_tri_sp;
    for(auto it = tri_considered.begin(); it != tri_considered.end(); it++)
    {
        std::array<boost::geometry::model::point<float, 3UL, boost::geometry::cs::spherical<boost::geometry::degree>>, 3UL> p_spherical;
        p_spherical = OcclusionCulling::convertToSphericalInVPSys(*it, tf);

        //Create Object to store spherical coordinates and reference to original object
        TriSpherical* tri_sp_temp = new TriSpherical;
        tri_sp_temp->tri_orig = *it;
        tri_sp_temp->vertices = p_spherical;
        float sum = 0;
        for(auto it2 = p_spherical.begin(); it2 != p_spherical.end(); it2++)
        {
            sum += (*it2).get<2>();
        }
        tri_sp_temp->mean_distance_to_vp = sum/p_spherical.size();
        if(std::abs(tri_sp_temp->mean_distance_to_vp) > 1000) continue;

        ordered_tri_sp[tri_sp_temp->mean_distance_to_vp] = tri_sp_temp;
    }

    //Create set to hold points with their respective triangles
    //Use Vector3f as key? Might create duplicates because of handling objects not references
    std::map<std::tuple<float, float, float>, PointSpherical*> points_sp;
    for(auto tri: tri_checked)
    {
        Eigen::Vector3f point;
        for(int i=0; i<3; i++)
        {
            switch(i)
            {
            case 0:
                point = tri->x1;
                break;
            case 1:
                point = tri->x2;
                break;
            case 2:
                point = tri->x3;
                break;
            }
            
            auto tuple = std::make_tuple(point[0],point[1], point[2]);
            if(points_sp.find(tuple) == points_sp.end())
            {
                //Initialize element
                PointSpherical* p = new PointSpherical;
                p->vertex = OcclusionCulling::convertToSphericalInVPSys(point, tf);
                points_sp[tuple] = p;
            }
            points_sp[tuple]->tris_attached.insert(tri);
        } 
    }


    for(auto it = ordered_tri_sp.begin(); it != ordered_tri_sp.end(); it++)
    {
        auto tri_sp_temp = it->second;
        std::array<bg::model::point<float, 3UL, bg::cs::spherical<bg::degree>>, 3UL> p_spherical = tri_sp_temp->vertices;

        //Compute spherical bounding box of triangle
        float angle_h_max, angle_v_max = -180.0;
        float angle_h_min, angle_v_min = 180.0;
        for(std::array<bg::model::point<float, 3, bg::cs::spherical<bg::degree> >, 3>::iterator it2 = p_spherical.begin(); it2 != p_spherical.end(); it2++)
        {
            //horizontal
            if((*it2).get<0>() > angle_h_max)
            {
                angle_h_max = (*it2).get<0>();
            }
            if((*it2).get<0>() < angle_h_min)
            {
                angle_h_min = (*it2).get<0>();
            }
            //vertical
            if((*it2).get<1>() > angle_v_max)
            {
                angle_v_max = (*it2).get<1>();
            }
            if((*it2).get<1>() < angle_v_min)
            {
                angle_v_min = (*it2).get<1>();
            }
        }

        int index_h_min = (int)(angle_h_min)/angular_increment_h;
        int index_h_max = (int)(angle_h_max)/angular_increment_h + 1;
        int index_v_min = (int)(angle_v_min)/angular_increment_v;
        int index_v_max = (int)(angle_v_max)/angular_increment_v + 1;

        //Iterate over relevant part of z-buffer
        for(int h=index_h_min; h<index_h_max; h++)
        {
            for(int v=index_v_min; v<index_v_max; v++)
            {
                std::pair<horiz_ind, vert_ind > raster_indices = std::make_pair(h, v);
                RasterElement raster_temp;
                //Check if element already exists and fetch it if so
                if(z_buffer.find(raster_indices) != z_buffer.end())
                {
                    //Element already exists
                    raster_temp = z_buffer[raster_indices];
                }
                
                //TO-DO_old: Check if it is worthwile to implement the covering mechanic again
                OcclusionCulling::fillRasterElement(raster_temp, tri_sp_temp, tri_to_return);

                z_buffer[raster_indices] = raster_temp;
            }
        }
    }

    //Iterate through point set to check if they are occluded
    for(auto point: points_sp)
    {

        for(auto tri: ordered_tri_sp)
        {
            //if facet is closer to camera than point
            if(tri.second->mean_distance_to_vp < point.second->vertex.get<2>())
            {
                if(OcclusionCulling::geometryIsWithin(dropZCoordinate(point.second->vertex), tri.second))
                {
                    point.second->is_occluded = true;
                    
                    break;
                }
            }
        }
    }
    
    tri_to_return = tri_checked;

    for(auto point: points_sp)
    {
        if(point.second->is_occluded)
        {
            for(auto tri: point.second->tris_attached)
            {
                tri_to_return.erase(tri);
            }
        }
    }

    //Clean up
    for(auto point: points_sp)
    {
        delete point.second;
    }
    points_sp.clear();

    for(auto tri: ordered_tri_sp)
    {
        delete tri.second;
    }
    ordered_tri_sp.clear();

    return tri_to_return;
}


void OcclusionCulling::fillRasterElement(RasterElement &ras_el, TriSpherical *tri_sp_under_test, std::unordered_set<tri_t*> &tri_spherical_to_return)
{
    ras_el.tri_contained.insert(tri_sp_under_test);
}

bool OcclusionCulling::geometryIsWithin(bg::model::point<float, 2, bg::cs::cartesian> point_under_test, TriSpherical* tri_sp)
{
    bg::model::polygon<bg::model::point<float, 2, bg::cs::cartesian>> poly;    //TO-DO_old: Can this just be converted to cartesian? Projection skewed? search "2, bg::cs" for occurences
    for(auto it = tri_sp->vertices.begin(); it != tri_sp->vertices.end(); it++)
    {
        bg::append(poly.outer(), dropZCoordinate(*it));
    }
    bg::append(poly.outer(), dropZCoordinate(*(tri_sp->vertices.begin()))); //Add first point to close
    return bg::within(point_under_test, poly);
}

void OcclusionCulling::occlusionCheck_GPU(std::map<std::tuple<float, float, float>, PointSpherical *> &points_checked, 
                            std::unordered_set<TriSpherical *> &tri_considered)
{
    //TO-DO: Rethink use of stl containers. Too slow? Specifically passing by value since copies are inefficient
    //TO-DO: Do not convert to spherical, do occlusion check as ray triangle intersection checking

    unsigned int size_tri = tri_considered.size();

    bool use_gpu;
    ros::param::get("~/algorithm/use_gpu", use_gpu);

    float *tri_distance;
    tri_distance = (float*)malloc(size_tri*sizeof(float));

    bool *result;
    float point_x, point_y;
    float *vertex0_x, *vertex0_y, *vertex1_x, *vertex1_y, *vertex2_x, *vertex2_y;

    //Allocate memory for c arrays
    vertex0_x = (float*)malloc(size_tri*sizeof(float));
    vertex0_y = (float*)malloc(size_tri*sizeof(float));
    vertex1_x = (float*)malloc(size_tri*sizeof(float));
    vertex1_y = (float*)malloc(size_tri*sizeof(float));
    vertex2_x = (float*)malloc(size_tri*sizeof(float));
    vertex2_y = (float*)malloc(size_tri*sizeof(float));
    result = (bool*)malloc(size_tri*sizeof(bool));
    
    //Fill c array to pass to the gpu
    int i=0;
    for(auto tri: tri_considered)
    {
        float max_dist = 0;
        //Use the maximum distance between triangle and vp as the distance value
        for(auto v: tri->vertices)
        {
            if(v.get<2>() > max_dist) max_dist = v.get<2>();
        }
        tri_distance[i] = max_dist;
        vertex0_x[i] = tri->vertices[0].get<0>();
        vertex0_y[i] = tri->vertices[0].get<1>();
        vertex1_x[i] = tri->vertices[1].get<0>();
        vertex1_y[i] = tri->vertices[1].get<1>();
        vertex2_x[i] = tri->vertices[2].get<0>();
        vertex2_y[i] = tri->vertices[2].get<1>();
        i++;
    }

    //Invoke function that handles GPU memory and launches CUDA-Kernels
    for(auto point: points_checked)
    {
        point_x = point.second->vertex.get<0>();
        point_y = point.second->vertex.get<1>();

        within_triangle_query_gpu_driver(size_tri,THREADS_PER_BLOCK,
                    point_x,
                    point_y,
                    vertex0_x,
                    vertex0_y,
                    vertex1_x,
                    vertex1_y,
                    vertex2_x,
                    vertex2_y,
                    result,
                    use_gpu);

        //Check if one of the result entries is occluded
        for(int i=0; i<size_tri; i++)
        {
            if((result[i] == true) && (point.second->vertex.get<2>() > tri_distance[i]))
            {
                point.second->is_occluded = true;
                break;
            }
        }
    }

    free(vertex0_x);
    free(vertex0_y);
    free(vertex1_x);
    free(vertex1_y);
    free(vertex2_x);
    free(vertex2_y);
    free(result);

    free(tri_distance);
}

void OcclusionCulling::occlusionCheck_usingBoost(std::map<std::tuple<float, float, float>, PointSpherical *> &points_checked, 
                            std::unordered_set<TriSpherical *> &tri_considered)
{
    // Iterate through point set to check if they are occluded
    for(auto point: points_checked)
    {
        for(auto tri: tri_considered)
        {
            //if facet is closer to camera than point
            if(tri->mean_distance_to_vp < point.second->vertex.get<2>())
            {
                if(OcclusionCulling::geometryIsWithin(dropZCoordinate(point.second->vertex), tri))
                {
                    point.second->is_occluded = true;
                    
                    break;
                }
            }
        }
    }
}

char OcclusionCulling::geometryIsWithin(TriSpherical* tri_sp_pot_ocludee, TriSpherical* tri_sp_occluder)
{
   
    bg::model::polygon<bg::model::point<float, 2, bg::cs::cartesian>> poly_occludee, poly_occluder;
    auto test1 = tri_sp_pot_ocludee->vertices.begin();
    auto test2 = tri_sp_pot_ocludee->vertices.end();
    
    for(auto it = tri_sp_pot_ocludee->vertices.begin(); it != tri_sp_pot_ocludee->vertices.end(); it++)
    {
        bg::append(poly_occludee.outer(), bg::model::point<float, 2, bg::cs::cartesian>((*it).get<0>(), (*it).get<1>()));
    }
    for(auto it = tri_sp_occluder->vertices.begin(); it != tri_sp_occluder->vertices.end(); it++)
    {
        bg::append(poly_occluder.outer(), bg::model::point<float, 2, bg::cs::cartesian>((*it).get<0>(), (*it).get<1>()));
    }

    if(bg::within(poly_occludee, poly_occluder)) 
    {
        return WITHIN;
    }
    else if(bg::intersects(poly_occludee, poly_occluder))
    {
        //Catch case when two neighbouring facets are taken as intersecting due to rounding inaccuracies
        //Calculate intersection area and return OUTSIDE if it is sufficiently small
        std::vector<bg::model::polygon<bg::model::point<float, 2, bg::cs::cartesian>>> poly_intersect;
        boost::geometry::intersection(poly_occludee, poly_occluder, poly_intersect);
        float area = 0;
        for(bg::model::polygon<bg::model::point<float, 2, bg::cs::cartesian>> p: poly_intersect)
        {
            area += bg::area(p);
        }
        if(area < 1)
        {
            return OUTSIDE;
        }
        return INTERSECTING;
    }
    else
    {
        return OUTSIDE;
    }
}

bg::model::point<float, 3, bg::cs::spherical<bg::degree> > OcclusionCulling::convertToSphericalInVPSys(Eigen::Vector3f point, tf2::Transform tf)
{
    //Convert Eigen::Vector3f to tf2::Vector3
    tf2::Vector3 tri_wcs, tri_vp;
    bg::model::point<float, 3, bg::cs::spherical<bg::degree> > p_spherical;
    tri_wcs.setX(point[0]); 
    tri_wcs.setY(point[1]); 
    tri_wcs.setZ(point[2]);

    //Perform coordinate transforms for all vertices
    //TO-DO_old: If converting to class, make tf member
    tri_vp = tf * tri_wcs;

    bg::model::point<float, 3, bg::cs::cartesian> p_cartesian (tri_vp.getX(), tri_vp.getY(), tri_vp.getZ());

    //Transform to sperical CS
    //Rotate in order below, so that z-axis points to the transformed point
    //input  = (x, y, z)
    //output = (angle about z axis, angle about new y axis, radius)
    //Points on cartesian z-axis have spherical (0, 0, radius) coordinates
    //(angle_h, angle_v, radius)
    bg::transform(p_cartesian, p_spherical);
    return p_spherical;
}

std::array<bg::model::point<float, 3, bg::cs::spherical<bg::degree> >, 3> OcclusionCulling::convertToSphericalInVPSys(tri_t* tri, tf2::Transform tf)
{
    std::array<Eigen::Vector3f, 3> input_arr;
    std::array<bg::model::point<float, 3, bg::cs::spherical<bg::degree> >, 3> output_arr;

    input_arr[0] = tri->x1;
    input_arr[1] = tri->x2;
    input_arr[2] = tri->x3;

    for(int i=0; i<input_arr.size(); i++)
    {
        output_arr[i] = convertToSphericalInVPSys(input_arr[i], tf);
    }
    
    return output_arr;
}

bool RasterElement::isCompletelyCovered()
{
    return this->is_completely_covered;
}

void RasterElement::setCompletelyCovered(bool x)
{
    this->is_completely_covered = x;
}

TriSpherical* RasterElement::getCoveringTriangle()
{
    return this->completely_covered_by;
}

void RasterElement::setCoveringTriangle(TriSpherical* tri_sp)
{
    this->completely_covered_by = tri_sp;
}

bg::model::point<float, 2, bg::cs::cartesian> dropZCoordinate(bg::model::point<float, 3, bg::cs::spherical<bg::degree>> convertee)
{
    return bg::model::point<float, 2, bg::cs::cartesian>(convertee.get<0>(), convertee.get<1>());
}