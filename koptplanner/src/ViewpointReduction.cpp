#ifndef __VIEWPOINTREDUCTION_CPP__
#define __VIEWPOINTREDUCTION_CPP__

#include "ViewpointReduction/ViewpointReduction.h"
#include "std_msgs/String.h"
#include <string> 

ViewpointReduction::ViewpointReduction(int vpCount)
{
  this->vpCount = vpCount;
  this->iteration = 0;
}

//TO-DO: Replace VPcount variable by a better solution to get the number of viewpoints
//TO-DO: Handle tri-Entries that correspond to required view points
//TO-DO: Different system to keep track of rows and columns
void ViewpointReduction::generateVisibilityMatrix(std::vector<tri_t*> tri, StateVector * VP)
{
  this->visMat.resize(tri.size(),this->vpCount);
  this->visMat.fill(false);

  // tri-Array holds not only mesh, but also fixed, given VPs, that the robot is supposed to hit during the tour. Filter those out
  // int noOfRequiredVPs = 0;
  // for (int i = 0; i < tri.size(); i++)
  // {
  //   if(tri.at(i)->Fixpoint)
  //   {
  //     noOfRequiredVPs++;
  //   }
  // }

  //Viewpoint-Counter, columns
  for(int i = 0; i < this->vpCount; i++)
  {
    //Triangle-Counter, rows
    //TO-DO: Use iterator
    for (int j = 0; j < tri.size(); j++)
    {
      bool isVisible = tri.at(j)->isVisible(VP[i]);
      this->visMat(j,i) = isVisible;
    }
  }

  #ifdef GENERATE_MATLAB_FILE
  if(this->iteration == 0)
  {
    this->exportMatlabData("VisibilityMatrix", VP, this->visMat.cols());
  }
  #endif

  this->solveSetCoveringProbGreedy();

  this->removeRedundantVPs(VP);
  
  #ifdef GENERATE_MATLAB_FILE
  if(this->iteration == 0)
  {
    this->exportMatlabData("VisibilityMatrix_afterReduction", VP, this->viewpoints_kept.size());
  }
  #endif

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

std::vector<int> ViewpointReduction::getVPsKept()
{
  return this->viewpoints_kept;
}

void ViewpointReduction::solveSetCoveringProbGreedy()
{
  std::vector<bool> triangleCovered(this->visMat.cols(), false);    //to keep track whether or not the facet seen by some VP
  
  bool allTrisCovered;
  int itn = 0;
  do
  {
    this->computeSumOfVisibleTriangles();
    
    //TO-DO: Improve handling of no longer converging VP-Reduction
    //Determine VP that sees most uncovered triangles
    int maxVisibleElementIndex;
    int oldIndex = maxVisibleElementIndex;
    maxVisibleElementIndex = std::max_element(this->sumsOfTriangles.begin(),this->sumsOfTriangles.end()) - this->sumsOfTriangles.begin();
    this->viewpoints_kept.push_back(maxVisibleElementIndex);
    if (oldIndex == maxVisibleElementIndex)
    {
      ROS_ERROR("Viewpoint Reduction no longer converging at iteration %i", itn);
      break;
    } 
    
    //looping over triangles
    for(int i=0; i<this->visMat.rows(); i++){
      if(this->visMat.coeff (i,maxVisibleElementIndex) == true)
      {
        triangleCovered.at(i) = true;
        
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
    for(int i=0; i<triangleCovered.size(); i++)
    {
      if(triangleCovered.at(i) == false) 
      {
        allTrisCovered = false;
        break;
      }
    }

    //DEBUG/////////////////////////////
    itn++;
    int noUnseen = 0;
    if (itn < 10)
    {
      std_msgs::String msg;
      std::stringstream ss;
      ss << "Triangles covered: ";

      for(int i=0; i<triangleCovered.size(); i++)
      {
        ss << triangleCovered.at(i);
      }
      msg.data = ss.str();

      //ROS_INFO("%s", msg.data.c_str());
    }
    for (int i=0; i<triangleCovered.size(); i++)
    {
      if(triangleCovered.at(i)==false) noUnseen++;
    }
    if(itn == 40) ROS_INFO("Unseen Triangle:\t%i", maxVisibleElementIndex);
    /////////////////////////////////////
  }while(allTrisCovered == false); //while there remains a "false" entry

  //Debug////////////////////////
  std::stringstream ss;
  ss << "Viewpoints kept:\t";
  for (int i=0; i<viewpoints_kept.size(); i++)
  {
    ss << viewpoints_kept.at(i) << "\t";
  }
  ROS_INFO("%s",ss.str().c_str());
  ///////////////////////////////
}

//TO-DO: Do not pass VP, use class variable
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
			if(this->viewpoints_kept.at(j) == iter) 
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
    //TO-DO: Use iterator
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