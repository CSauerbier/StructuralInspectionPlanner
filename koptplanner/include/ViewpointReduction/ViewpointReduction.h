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

class ViewpointReduction
{
private:
  Eigen::Array<bool, Dynamic, Dynamic> visMat;  //Data-Structure to store visibility matrix
  std::vector<int> sumsOfTriangles;
  int vpCount;
  std::vector<int> viewpoints_kept;             //Viewpoints that are kept as a minimum set
  int iteration;
public:
  ViewpointReduction(int);

  void generateVisibilityMatrix(std::vector<tri_t*> tri, StateVector * VP);
  void computeSumOfVisibleTriangles();
  Eigen::Array<bool,Dynamic,Dynamic> getVisMat();
  std::vector<int> getSumOfTriangles();
  int getNoOfUniqueVPs();
  std::vector<int> getVPsKept();
  void solveSetCoveringProbGreedy();
  void removeRedundantVPs(StateVector * VP);
  void exportMatlabData(std::string fname, StateVector * VP, int noOfVPs);
};

void removeRow(Eigen::Array<bool,Dynamic,Dynamic>& matrix, unsigned int rowToRemove);
void removeColumn(Eigen::Array<bool,Dynamic,Dynamic>& matrix, unsigned int colToRemove);

#endif