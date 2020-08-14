#ifndef __VIEWPOINTREDUCTION_H__
#define __VIEWPOINTREDUCTION_H__

#include "ros/ros.h"
#include "koptplanner/TriangleObject.h"
//#include <eigen3/Eigen/Dense>
//#include "koptplanner/plan.hpp"
#include "koptplanner/ptpPlanner.hpp" //TODO: tidy up include directives
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
public:
  ViewpointReduction(int);

  void generateVisibilityMatrix(std::vector<tri_t*> tri, StateVector * VP, int iter);
  void computeSumOfVisibleTriangles();
  Eigen::Array<bool,Dynamic,Dynamic> getVisMat();
  std::vector<int> getSumOfTriangles();
};

#endif