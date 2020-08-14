#ifndef __VIEWPOINTREDUCTION_CPP__
#define __VIEWPOINTREDUCTION_CPP__

#include "ViewpointReduction/ViewpointReduction.h"

ViewpointReduction::ViewpointReduction(int vpCount)
{
  this->vpCount = vpCount;
}

//TODO: Replace VPcount variable by a better solution to get the number of viewpoints
//TODO: Write separate function for Matlab-File-generation
//TODO: Write class for visibility Matrix storage in object
//TODO: Handle tri-Entries that correspond to required view points
//TODO: Different system to keep track of rows and columns
void ViewpointReduction::generateVisibilityMatrix(std::vector<tri_t*> tri, StateVector * VP, int iter)
{
  std::string pkgPath = ros::package::getPath("koptplanner");
  std::fstream plannerLog;
  plannerLog.open((pkgPath+"/data/VisibilityMatrix.m").c_str(), std::ios::app | std::ios::out);
  if(!plannerLog.is_open())
    ROS_ERROR("Could not open VisibiltyMatrix.log");
  
  std::stringstream ss;
  ss << iter;
  std::string iterStr = ss.str();
  plannerLog << "VisibilityMatrix"+iterStr+" = [\n";
  
  this->visMat.resize(tri.size(),this->vpCount);
  this->visMat.fill(false);


  //Viewpoint-Counter, columns
  for(int i = 0; i < this->vpCount; i++)
  {
    //Triangle-Counter, rows
    //TODO: Use iterator
    for (int j = 0; j < tri.size(); j++)
    {
      bool isVisible = tri.at(j)->isVisible(VP[i]);
      plannerLog << (int)isVisible << ",\t";
      this->visMat(j,i) = isVisible;
    }
    plannerLog << ";\n";
  }
  plannerLog << "];\n";
  plannerLog.close();

  this->computeSumOfVisibleTriangles();
}

void ViewpointReduction::computeSumOfVisibleTriangles()
{
  //Iterate through Viewpoints
  for (int i=0; i<this->vpCount; i++)
  {
    this->sumsOfTriangles.push_back(visMat.col(i).count()); //Count number of visible facets for each view point 
  }
}

Eigen::Array<bool,Dynamic,Dynamic> ViewpointReduction::getVisMat()
{
  return this->visMat;
}

std::vector<int> ViewpointReduction::getSumOfTriangles()
{
  return this->sumsOfTriangles;
}

#endif