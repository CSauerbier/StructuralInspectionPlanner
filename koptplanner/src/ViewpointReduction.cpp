#ifndef __VIEWPOINTREDUCTION_CPP__
#define __VIEWPOINTREDUCTION_CPP__

#include "ViewpointReduction/ViewpointReduction.h"
#include "Visualization/Visualization.h"    //TO-DO
#include <vector>

ViewpointReduction::ViewpointReduction(int vpCount)
{
  this->vpCount = vpCount;
  this->iteration = 0;
}

//TO-DO: Replace VPcount variable by a better solution to get the number of viewpoints
//TO-DO: Handle tri-Entries that correspond to required view points (Member variable "Fixpoint" = true)
//TO-DO: Different system to keep track of rows and columns
//TO-DO: Make more robust, e.g. by initializing all member variables 
//TO-DO: LKH Error "DIMENSION < 3 or not specified" occurs when no triangle is visible
void ViewpointReduction::generateVisibilityMatrix(std::vector<tri_t*> tri, StateVector * VP)
{
  this->triangles = tri;
  this->view_points = VP;

  this->visMat.resize(tri.size(),this->vpCount);
  this->visMat.fill(false);
  
  int progress_old = 0;
  //Viewpoint-Counter, columns
  for(int i = 0; i < this->vpCount; i++)
  {
    int progress = (int)i*100/this->vpCount;
    if(progress != progress_old) ROS_INFO("Progress: %i\t%%", progress);
    progress_old = progress;
    std::unordered_set<tri_t*> vis_set(tri.begin(), tri.end());

    vis_set = FrustumCulling::getFacetsWithinFrustum(vis_set, VP[i], true);

    vis_set = BackfaceCulling::getFrontFacets(VP[i], vis_set);

    vis_set = OcclusionCulling::getUnoccludedFacets(VP[i], vis_set, vis_set);

    StateVector vp_tmp = (VP[i]);
    //Triangle-Counter, rows
    for (int j = 0; j < tri.size(); j++)
    {
      if(vis_set.find(tri.at(j)) != vis_set.end())
      {
        this->visMat(j,i) = tri.at(j)->isVisible(VP[i]);
      }
      else
      {
        this->visMat(j,i) = false;
      }
    }
  }

  #ifdef GENERATE_MATLAB_FILE
  if(this->iteration == 0)
  {
    this->exportMatlabData("VisibilityMatrix", VP, this->visMat.cols());
  }
  #endif

  this->solveSetCoveringProbGreedy();

  this->iteration ++;
}

void ViewpointReduction::computeSumOfVisibleTriangles()
{
  this->sumsOfTriangles.clear();
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

int ViewpointReduction::getNoOfUniqueVPs()
{
  return this->viewpoints_kept.size();
}

std::vector<VisibilityContainer> ViewpointReduction::getVPsKept()
{
  return this->viewpoints_kept;
}

std::vector<tri_t *> ViewpointReduction::getUncoveredTriangles()
{
  return this->uncovered_triangles;
}

void ViewpointReduction::solveSetCoveringProbGreedy()
{
  std::vector<bool> triangleCovered(this->visMat.rows(), false);    //to keep track whether or not the facet seen by some VP
  
  bool allTrisCovered;  //TO-DO: Only keep one of these
  int no_uncovered_tris = 0;
  int itn = 0;
  do
  {
    this->computeSumOfVisibleTriangles();
    
    //TO-DO: Improve handling of no longer converging VP-Reduction
    //Determine VP that sees most uncovered triangles
    int maxVisibleElementIndex;
    int oldIndex = maxVisibleElementIndex;
    maxVisibleElementIndex = std::max_element(this->sumsOfTriangles.begin(),this->sumsOfTriangles.end()) - this->sumsOfTriangles.begin();
    if (oldIndex == maxVisibleElementIndex)
    {
      for(int i=0; i<triangleCovered.size(); i++)
      {
        if(triangleCovered.at(i) == false) 
        {
          this->uncovered_triangles.push_back(this->triangles.at(i));
        }
      }
      ROS_INFO("Viewpoint Reduction no longer converging at iteration %i, %i uncovered facets", itn, no_uncovered_tris);
      break;
    } 
    
    std::vector<tri_t*> tris_temp;
    //looping over triangles
    for(int i=0; i<this->visMat.rows(); i++){
      if(this->visMat.coeff (i,maxVisibleElementIndex) == true)
      {
        triangleCovered.at(i) = true;
        tris_temp.push_back(this->triangles.at(i));
        
        //Once a facet is seen by a VP that is already selected, set the row to false
        //so that it does not contribute to the score of next VPs
        for(int j=0; j<this->visMat.cols(); j++)
        {
          this->visMat(i,j) = false;
        }
      }
    }

    //Check if all facets are seen by some VP
    allTrisCovered = true;
    no_uncovered_tris = 0;
    for(int i=0; i<triangleCovered.size(); i++)
    {
      if(triangleCovered.at(i) == false) 
      {
        allTrisCovered = false;
        no_uncovered_tris++;
      }
    }

    VisibilityContainer vc_temp;
    vc_temp.set(maxVisibleElementIndex, &this->view_points[maxVisibleElementIndex], tris_temp);
    this->viewpoints_kept.push_back(vc_temp);

    itn++;
  }while(allTrisCovered == false);

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

//TO-DO: Do not pass VP, use member variable
//TO-DO: Possibly use set for VP reference
void ViewpointReduction::removeRedundantVPs(StateVector * VP)
{
	int iter = 0;
  for(int i=0; i<this->vpCount; i++)
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
			for (int j=i; j<(this->vpCount-1); j++)
			{
				VP[j] = VP[j+1];
			}
			//VP[this->vpCount].setZero();
			this->vpCount--;
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
  for(int i = 0; i < this->visMat.cols(); i++)
  {
    //Triangle-Counter, rows
    for (int j = 0; j < (this->visMat.rows()); j++)
    {
      plannerLog << (int)this->visMat(j,i) << ",\t"; //row,col
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

VisibilityContainer::~VisibilityContainer()
{
}

void VisibilityContainer::set(int vp_num, StateVector *VP, std::vector<tri_t*> tri)
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

//Convenience functions to remove rows/columns from Eigen-Matrix
void removeRow(Eigen::Array<bool,Dynamic,Dynamic>& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}
//Convenience functions to remove rows/columns from Eigen-Matrix
void removeColumn(Eigen::Array<bool,Dynamic,Dynamic>& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

#endif