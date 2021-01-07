#ifndef __VIEWPOINTREDUCTION_H__
#define __VIEWPOINTREDUCTION_H__

#include "ros/ros.h"
#include "koptplanner/TriangleObject.h"
#include "koptplanner/ptpPlanner.hpp"
#include <stdlib.h>
#include <fstream>
#include <ros/package.h>
#include <sstream>
#include <vector>
#include <unordered_set>
#include "Culling/OcclusionCulling.h"
#include "Culling/BackfaceCulling.h"
#include "Culling/FrustumCulling.h"
#include "Visualization/Visualization.h"
#include "Culling/DistanceCulling.h"
#include "Culling/Coordinates.hpp"
#include <map>
#include <math.h>
#include <time.h>
#include <stdexcept>


#ifdef __TIMING_INFO__
extern long time_occlusion_query;
extern long time_visibility_determination;
#endif

typedef bool vis_mat_entry_t;

/**
 * Utility class to provide more convenient access to a visibility matrix structure that arranges 
 * facet visibility information for all view points in a matrix
 * */
class VisibilityMatrix
{
private:
    Eigen::Array<vis_mat_entry_t, Dynamic, Dynamic> matrix;
public:
    /**
     * Initializes the object by setting the dimensions of the matrix
     * \param tri_size Number of mesh facets. Each row represents a facet. 
     * \param vp_size Number of view points. Each column represents a view point. 
     * */
    void init(size_t tri_size, size_t vp_size);

    /**
     * Returns the visibility information stored as one element of the visibility matrix
     * \param tri Facet number to be queried
     * \param vp View point number to be queried
     * \returns Visibility information on the facet tri when viewed from vp
     * */
    bool getEntry(size_t tri, size_t vp);

    /**
     * Sets one entry of the visibility matrix
     * \param tri Facet number to be set
     * \param vp View point number to be set
     * \param set_value Value to be entered at the specified position
     * */
    void setEntry(size_t tri, size_t vp, vis_mat_entry_t set_value);

    /**
     * \returns Number of view points as specified in init(). (Equals the number of columns of the visibility matrix)
     * */
    size_t getNoVPs();
    
    /**
     * \returns Number of facets as specified in init(). (Equals the number of rows of the visibility matrix)
     * */
    size_t getNoTris();
};


/**
 * Class is to hold information on the view points and the correspoinding visible surfaces from them, 
 * excluding those already visible from a previously selected view point
 * */
class VisibilityContainer
{
private:
    int vp_number;
    StateVector view_point;
    std::vector<tri_t*> triangle_vector;
public:
    /**
     * Constructor to set all VisibilityContainer member variables.
     * */
    VisibilityContainer(int vp_num, StateVector *VP, std::vector<tri_t*> tri);

    /**
     * \returns Index of the view point in the original c-style array represented by the container
     * */
    int getVPNum();

    /**
     * \returns Coordinates of the corresponding view point as an Eigen::Matrix<float, 5, 1>
     * */
    StateVector* getVP();

    /**
     * \returns Mesh elements visible from the respective view point and not already seen by another 
     * previously selected view point
     * */
    std::vector<tri_t*> getTriVect();
};


/**
 * Class that computes mesh visibility for view points, taking into account occlusion, 
     * and solves the set cover problem. The results can be accesed through the public member functions.
 * */
class ViewpointReduction
{
private:
    VisibilityMatrix vis_matrix;
    int vp_count;
    std::vector<VisibilityContainer> viewpoints_kept;   //Viewpoints that are kept as a minimum set
    int iteration;
    std::vector<tri_t *> triangles;
    std::vector<tri_t *> triangles_considered;
    std::vector<tri_t *> uncovered_triangles;
    StateVector * view_points;
    std::vector<float> triangle_surface_areas;
    float surface_area;
    float area_stop_criterion;

    void generateVisibilityMatrix();
    void solveSetCoveringProbGreedy();
    void exportMatlabData(std::string fname, StateVector * VP, int noOfVPs);
    void setTriangleSurfaceAreas();
    int findNextBestVP();
    int findNextBestVP(std::vector<bool> &triangle_covered);
    float computeSurfaceArea(std::vector<tri_t*> tri_vct);
    void setUncoveredTriangles(std::vector<bool>);
public:
    /** 
     * Creates a ViewpointReduction-Object that processes mesh visibility for view points, taking into account occlusion, 
     * and solves the set cover problem. The results can be accesed through the public member functions.
     * \param tri_checked Mesh for which maximum visibility is to be a achieved
     * \param tri_considered tri_checked elements plus additional mesh elements considered for occlusion, but not required to be visible
     * \param VP Pointer to C-Style Array holding view points as StateVector elements
     * \param vp_count Number of elements to be considered in VP. Must not exceed actual number of array elements in VP
     * */
    ViewpointReduction(std::vector<tri_t*> tri_checked, std::vector<tri_t*> tri_considered, StateVector * VP, int vp_count);

    /**
     * \returns Number of viewpoints selected to achieve the best coverage of the mesh
     * */
    int getNoOfSelectedVPs();

    /**
     * \returns Set of view points best covering the mesh as a result of set cover problem solver 
     * */
    std::vector<VisibilityContainer> getSelectedVPs();

    /**
     * \returns Mesh elements that are not visible from any view point in the selected best covering set
     * */
    std::vector<tri_t *> getUncoveredTriangles();

    /**
     * Modifies the C-Style array passed to the constructor so that the view points within the selected set
     * are reordered to be the first entries in front of those not selected
     * */
    void removeRedundantVPs();
};

#endif