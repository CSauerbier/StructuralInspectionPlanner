#ifndef __VIEWPOINTREDUCTION_H__
#define __VIEWPOINTREDUCTION_H__

//TO-DO: add ROS-parameter for this
// define if a .m output file is to be generated
// #define GENERATE_MATLAB_FILE

#include "ros/ros.h"
#include "koptplanner/TriangleObject.h"
#include "koptplanner/ptpPlanner.hpp" //TO-DO: tidy up include directives
#include <stdlib.h>
#include <fstream>
#include <ros/package.h>
#include <sstream>
#include <vector>

class VisibilityContainer
{
private:
  int vp_number;
  StateVector view_point;
  std::vector<tri_t*> triangle_vector;
public:
  ~VisibilityContainer();

  void set(int vp_num, StateVector *VP, std::vector<tri_t*> tri);
  int getVPNum();
  StateVector* getVP();
  std::vector<tri_t*> getTriVect();
};


class ViewpointReduction
{
private:
  Eigen::Array<bool, Dynamic, Dynamic> visMat;  //Data-Structure to store visibility matrix
  std::vector<int> sumsOfTriangles;
  int vpCount;
  std::vector<VisibilityContainer> viewpoints_kept;             //Viewpoints that are kept as a minimum set
  int iteration;
  std::vector<tri_t *> triangles;
  StateVector * view_points;
public:
  ViewpointReduction(int);

  void generateVisibilityMatrix(std::vector<tri_t*> tri, StateVector * VP);
  void computeSumOfVisibleTriangles();
  Eigen::Array<bool,Dynamic,Dynamic> getVisMat();
  std::vector<int> getSumOfTriangles();
  int getNoOfUniqueVPs();
  std::vector<VisibilityContainer> getVPsKept();
  void solveSetCoveringProbGreedy();
  void removeRedundantVPs(StateVector * VP);
  void exportMatlabData(std::string fname, StateVector * VP, int noOfVPs);
};

void removeRow(Eigen::Array<bool,Dynamic,Dynamic>& matrix, unsigned int rowToRemove);
void removeColumn(Eigen::Array<bool,Dynamic,Dynamic>& matrix, unsigned int colToRemove);

#endif