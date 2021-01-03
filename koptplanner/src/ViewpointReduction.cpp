#include "ViewpointReduction/ViewpointReduction.h"


void VisibilityMatrix::init(size_t tri_size, size_t vp_size)
{
    this->matrix.resize(tri_size,vp_size);
    this->matrix.fill(false);
}

bool VisibilityMatrix::getEntry(size_t tri, size_t vp)
{
    return this->matrix(tri, vp);
}

void VisibilityMatrix::setEntry(size_t tri, size_t vp, vis_mat_entry_t set_value)
{
    this->matrix(tri, vp) = set_value;
}

size_t VisibilityMatrix::getNoVPs()
{
    return this->matrix.cols();
}

size_t VisibilityMatrix::getNoTris()
{
    return this->matrix.rows();
}


ViewpointReduction::ViewpointReduction(std::vector<tri_t*> tri_checked, std::vector<tri_t*> tri_considered, StateVector * VP, int vp_count)
{
    this->vp_count = vp_count;
    this->iteration = 0;
    this->triangles = tri_checked;
    this->triangles_considered = tri_considered;
    this->setTriangleSurfaceAreas();
    this->view_points = VP;
    this->vis_matrix.init(tri_checked.size(), this->vp_count);
    this->surface_area = this->computeSurfaceArea(this->triangles);
    ros::param::get("~/algorithm/stop_criterion", this->area_stop_criterion);
    this->area_stop_criterion /= 100.0; //Percent

    this->generateVisibilityMatrix();

    this->solveSetCoveringProbGreedy();
}

//TO-DO: Handle tri-Entries that correspond to required view points (Member variable "Fixpoint" = true)
void ViewpointReduction::generateVisibilityMatrix()
{
    bool moeller_trumbore_occlusion_check;
    ros::param::get("~/algorithm/moeller_trumbore_occlusion_check", moeller_trumbore_occlusion_check);

    #ifdef __TIMING_INFO__
    timeval time;
    gettimeofday(&time, NULL);
    time_visibility_determination -= time.tv_sec * 1000000 + time.tv_usec;
    #endif

    if(moeller_trumbore_occlusion_check)
    {    
        //Holds a single instance of all mesh vertices with the sum of their coordinate components as key.
        //TO-DO: Identification is not unique
        std::map<double, CartesianCoordinates*> vertices_map;

        //Keeps track of which triangles comprise a given point
        std::map<double, std::unordered_set<tri_t*>> vertex_triangle_links;

        //To store all triangle objects created
        std::vector<TriangleVertices*> tri_vert_vect;

        //Holds all objects created to assure complete deletion
        std::vector<CartesianCoordinates*> coord_vect;

        //Stores all vertices of the mesh in their respective data structures
        for(auto tri: this->triangles)
        {
            std::array<CartesianCoordinates*, 3> vertices;
            //Create dynamic instances to allow for object copy
            vertices[0] = new CartesianCoordinates(tri->x1[0], tri->x1[1], tri->x1[2]);
            vertices[1] = new CartesianCoordinates(tri->x2[0], tri->x2[1], tri->x2[2]);
            vertices[2] = new CartesianCoordinates(tri->x3[0], tri->x3[1], tri->x3[2]);
            tri_vert_vect.push_back(new TriangleVertices(*vertices[0], *vertices[1], *vertices[2]));

            for(auto vertex: vertices)
            {
                coord_vect.push_back(vertex);
                double key = vertex->x + vertex->y + vertex->z;
                //Insert to map if not already present
                vertices_map[key] = vertex;

                //Add triangle to the list of triangles attached to this point
                vertex_triangle_links[key].insert(tri);
            }
        }

        //Transfer map contents to vector of unique vertices
        std::vector<CartesianCoordinates*> vertices_vect;
        for(auto entry: vertices_map)
        {
            vertices_vect.push_back(entry.second);
        }

        //Convert view point array to fit interface
        std::vector<CartesianCoordinates*> vp_vect(this->vp_count, nullptr);
        for(int i=0; i<this->vp_count; i++)
        {
            vp_vect[i] = new CartesianCoordinates(this->view_points[i][0], this->view_points[i][1], this->view_points[i][2]);
        }
        
        int progress_old = 0;
        bool display_progress, use_gpu;
        ros::param::get("~/display/progress", display_progress);
        ros::param::get("~/algorithm/use_gpu", use_gpu);

        OcclusionCulling::initializeMoellerTrumbore(tri_vert_vect, vertices_vect, vp_vect, use_gpu);

        for(int vp = 0; vp < this->vp_count; vp++)
        {
            if(display_progress)
            {
                int progress = (int)vp*100/this->vp_count;
                if(progress != progress_old) ROS_INFO("Progress: %i\t%%", progress);
                progress_old = progress;
            }

            auto view_point = this->view_points[vp];

            #ifdef __TIMING_INFO__
            timeval time_occlusion;
            gettimeofday(&time_occlusion, NULL);
            time_occlusion_query -= time_occlusion.tv_sec * 1000000 + time_occlusion.tv_usec;
            #endif

            //Call CUDA function to solve the occlusion query
            std::vector<bool> points_occlusion_result = OcclusionCulling::occlusionCheck_GPU_MoellerTrumbore(vp);

            #ifdef __TIMING_INFO__
            gettimeofday(&time_occlusion, NULL);
            time_occlusion_query += time_occlusion.tv_sec * 1000000 + time_occlusion.tv_usec;
            #endif

            if(points_occlusion_result.size() != vertices_vect.size()) std::runtime_error("Size mismatch");

            //Assign visibility information to corresponding triangles. Using set to avoid duplicates. Initializing with all triangles considered
            std::unordered_set<tri_t*> visible_triangles(this->triangles.begin(), this->triangles.end());
            for(int i=0; i<vertices_vect.size(); i++)
            {
                //If the vertex is occluded, erase all the triangles attached to it from the visible_triangles vector
                if(!points_occlusion_result[i])
                {
                    auto vertex = vertices_vect[i];
                    float key = vertex->x + vertex->y + vertex->z;
                    auto linked_tris = vertex_triangle_links[key];
                    for(auto tri: linked_tris)
                    {
                        visible_triangles.erase(tri);
                    }
                }
            }

            //Boolean vector whose entries state whether the triangle at the same index is visible or not
            std::vector<bool> triangle_occlusion_result(this->triangles.size(), false);
            for(int i=0; i<this->triangles.size(); i++)
            {
                if(visible_triangles.find(this->triangles[i]) != visible_triangles.end())
                {
                    //Triangle at index i in original aray is visible
                    triangle_occlusion_result[i] = true;
                }
            }   

            //Transfer result to visibility matrix
            for (int tri = 0; tri < this->triangles.size(); tri++)
            {
                if(triangle_occlusion_result[tri])
                {
                    this->vis_matrix.setEntry(tri, vp, this->triangles.at(tri)->isVisible(this->view_points[vp]));
                }
                else
                {
                    this->vis_matrix.setEntry(tri, vp, false);
                }
            }
        }
        //Delete data structures for occlusion query
        OcclusionCulling::finalizeMoellerTrumbore();

        //Delete all objects
        for(auto coord: coord_vect)
        {
            delete coord;
            coord=NULL;
        }
        for(auto vp: vp_vect)
        {
            delete vp;
            vp = NULL;
        }
    }

    else
    {    
        int progress_old = 0;
        bool display_progress;
        ros::param::get("~/display/progress", display_progress);
        //Viewpoint-Counter, columns
        for(int vp = 0; vp < this->vp_count; vp++)
        {
            if(display_progress)
            {
                int progress = (int)vp*100/this->vp_count;
                if(progress != progress_old) ROS_INFO("Progress: %i\t%%", progress);
                progress_old = progress;
            }
            
            std::unordered_set<tri_t*> vis_set(this->triangles.begin(), this->triangles.end());
            std::unordered_set<tri_t*> vis_set_within_range;

            vis_set = FrustumCulling::getFacetsWithinFrustum(vis_set, this->view_points[vp], true);

            vis_set = BackfaceCulling::getFrontFacets(this->view_points[vp], vis_set);

            vis_set_within_range = DistanceCulling::getFacetsWithinDistance(vis_set, this->view_points[vp]);

            #ifdef __TIMING_INFO__
            timeval time_occlusion;
            gettimeofday(&time_occlusion, NULL);
            time_occlusion_query -= time_occlusion.tv_sec * 1000000 + time_occlusion.tv_usec;
            #endif

            vis_set = OcclusionCulling::getUnoccludedFacets(this->view_points[vp], vis_set_within_range, vis_set);

            #ifdef __TIMING_INFO__
            gettimeofday(&time_occlusion, NULL);
            time_occlusion_query += time_occlusion.tv_sec * 1000000 + time_occlusion.tv_usec;
            #endif

            for (int tri = 0; tri < this->triangles.size(); tri++)
            {
                if(vis_set.find(this->triangles.at(tri)) != vis_set.end())
                {
                    this->vis_matrix.setEntry(tri, vp, this->triangles.at(tri)->isVisible(this->view_points[vp]));
                }
                else
                {
                    this->vis_matrix.setEntry(tri, vp, false);
                }
            }
        }
    }

    #ifdef GENERATE_MATLAB_FILE
    if(this->iteration == 0)
    {
        this->exportMatlabData("VisibilityMatrix", VP, this->vis_matrix.cols());
    }
    #endif

    this->iteration ++;

    #ifdef __TIMING_INFO__
    gettimeofday(&time, NULL);
    time_visibility_determination += time.tv_sec * 1000000 + time.tv_usec;
    #endif
}

int ViewpointReduction::getNoOfSelectedVPs()
{
    return this->viewpoints_kept.size();
}

std::vector<VisibilityContainer> ViewpointReduction::getSelectedVPs()
{
    return this->viewpoints_kept;
}

std::vector<tri_t *> ViewpointReduction::getUncoveredTriangles()
{
    return this->uncovered_triangles;
}

void ViewpointReduction::solveSetCoveringProbGreedy()
{
    std::vector<bool> triangle_covered(this->vis_matrix.getNoTris(), false);        //to keep track whether or not the facet seen by some VP
    
    bool all_tris_covered;
    int no_uncovered_tris = 0;
    int itn = 0;
    do
    {
        // this->computeSumOfVisibleTriangles();    //TO-DO
        
        //TO-DO: Improve handling of no longer converging VP-Reduction
        //Determine VP that sees most surface area
        int max_area_vp;
        int oldIndex = max_area_vp;
        max_area_vp = this->findNextBestVP();
        if (oldIndex == max_area_vp)
        {
            for(int i=0; i<triangle_covered.size(); i++)
            {
                if(triangle_covered.at(i) == false) 
                {
                    this->uncovered_triangles.push_back(this->triangles.at(i));
                }
            }
            ROS_INFO("Viewpoint Reduction no longer converging at iteration %i, %i uncovered facets", itn, no_uncovered_tris);
            break;
        } 

        // Stop criterion for if VP information gain is below a threshold
        //looping over triangles
        std::vector<tri_t*> tris_seen;
        for(int tri=0; tri<this->vis_matrix.getNoTris(); tri++){
            if(this->vis_matrix.getEntry(tri,max_area_vp) == true)
            {
                tris_seen.push_back(this->triangles.at(tri));
            }
        }
        if(this->computeSurfaceArea(tris_seen) < this->surface_area*this->area_stop_criterion)
        {
            for(int i=0; i<triangle_covered.size(); i++)
            {
                if(triangle_covered.at(i) == false) 
                {
                    this->uncovered_triangles.push_back(this->triangles.at(i));
                }
            }
            ROS_INFO("Stop criterion reached at iteration %i, %i uncovered facets", itn, no_uncovered_tris);
            break;
        }

        std::vector<tri_t*> tris_temp;
        //looping over triangles
        for(int tri=0; tri<this->vis_matrix.getNoTris(); tri++){
            if(this->vis_matrix.getEntry(tri,max_area_vp) == true)
            {
                triangle_covered.at(tri) = true;
                tris_temp.push_back(this->triangles.at(tri));
                
                //Once a facet is seen by a VP that is already selected, set the row to false
                //so that it does not contribute to the score of next VPs
                for(int vp=0; vp<this->vis_matrix.getNoVPs(); vp++)
                {
                    this->vis_matrix.setEntry(tri,vp, false);
                }
            }
        }

        //Check if all facets are seen by some VP
        all_tris_covered = true;
        no_uncovered_tris = 0;
        for(int i=0; i<triangle_covered.size(); i++)
        {
            if(triangle_covered.at(i) == false) 
            {
                all_tris_covered = false;
                no_uncovered_tris++;
            }
        }

        VisibilityContainer vc_temp(max_area_vp, &this->view_points[max_area_vp], tris_temp);
        this->viewpoints_kept.push_back(vc_temp);

        itn++;
    }while(all_tris_covered == false);

    //Debug////////////////////////
    std::stringstream ss;
    ss << "Viewpoints kept:\t";
    for (int i=0; i<viewpoints_kept.size(); i++)
    {
        ss << viewpoints_kept.at(i).getVPNum() << "\t";
    }
    ROS_INFO("%s",ss.str().c_str());
    ///////////////////////////////
}

void ViewpointReduction::removeRedundantVPs()
{
	int iter = 0;
    for(int i=0; i<this->vp_count; i++)
    {
		//Check if queried VP belongs to minimal set
		bool isPartOfSet = false;
		for(int j=0; j<this->viewpoints_kept.size(); j++)
		{
			if(this->viewpoints_kept.at(j).getVPNum() == iter) 
			{
				isPartOfSet = true;
			}
		}
		
		//If the VP is not part of the minimum set, remove it and rearrange others
		if(!isPartOfSet)
		{
			for (int j=i; j<(this->vp_count-1); j++)
			{
				this->view_points[j] = this->view_points[j+1];
			}
			//VP[this->vpCount].setZero();
			this->vp_count--;
			i--;
		}
		iter++;
    }
}


void ViewpointReduction::exportMatlabData(std::string fname, StateVector *VP, int noOfVPs)
{
    std::string pkgPath = ros::package::getPath("koptplanner");
    std::fstream plannerLog;
    plannerLog.open((pkgPath+"/data/"+fname+".m").c_str(), std::ios::app | std::ios::out);
    if(!plannerLog.is_open())
        ROS_ERROR("Could not open .m file");
    
    std::stringstream ss;
    ss << iteration;
    std::string iterStr = ss.str();

    //Visibility-Matrix
    plannerLog << "VisibilityMatrix"+iterStr+" = [\n";
    //Viewpoint-Counter, columns
    for(int vp = 0; vp < this->vis_matrix.getNoVPs(); vp++)
    {
        //Triangle-Counter, rows
        for (int tri = 0; tri < (this->vis_matrix.getNoTris()); tri++)
        {
            plannerLog << (int)this->vis_matrix.getEntry(tri,vp) << ",\t"; //row,col
        }
        plannerLog << ";\n";
    }
    plannerLog << "];\n";

    //VPs
    plannerLog << "VPs"+iterStr+" = [\n";
    //Viewpoint-Counter, columns
    for(int i = 0; i < noOfVPs; i++)
    {
        //VP-Element Counter
        for (int j = 0; j < (int)VP->size(); j++)
        {
            plannerLog << VP[i][j] << ",\t"; //row,col
        }
        plannerLog << ";\n";
    }
    plannerLog << "];\n";
    plannerLog.close();
}

void ViewpointReduction::setTriangleSurfaceAreas()
{
    for(auto tri: this->triangles)
    {
        this->triangle_surface_areas.push_back(tri->a.norm());
    }
}

int ViewpointReduction::findNextBestVP()
{  
    std::vector<float> vp_area_covered(this->vp_count, 0.0);

    //Viewpoint-Counter, columns
    for(int i=0; i<this->vp_count; i++)
    {
        //Triangle-Counter, rows
        for(int j=0; j<this->triangles.size(); j++)
        {
            if(this->vis_matrix.getEntry(j,i))
            {
                vp_area_covered.at(i) += this->triangle_surface_areas.at(j);
            }
        }
    }
    return std::max_element(vp_area_covered.begin(),vp_area_covered.end()) - vp_area_covered.begin();
}

float ViewpointReduction::computeSurfaceArea(std::vector<tri_t*> tri_vct)
{
    float sum = 0;
    for(auto tri: tri_vct)
    {
        sum += tri->a.norm();
    }
    return sum;
}


VisibilityContainer::VisibilityContainer(int vp_num, StateVector *VP, std::vector<tri_t*> tri)
{
    this->vp_number = vp_num;
    this->view_point = *VP;
    this->triangle_vector = tri;
}

int VisibilityContainer::getVPNum()
{
    return this->vp_number;
}

StateVector* VisibilityContainer::getVP()
{
    return &this->view_point;
}

std::vector<tri_t*> VisibilityContainer::getTriVect()
{
    return this->triangle_vector;
}