#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "koptplanner/ptpPlanner.hpp"
#include "Visualization/Colors.h"
#include "Culling/Coordinates.hpp"

template <typename T>
T& Singleton() 
{
  static T single;
  return single;
}

//TO-DO: Improve color handling
class Visualization
{
protected:
  static int s_publishing_number;

  Visualization();
  Visualization(const Visualization&);  //Prohibit instanciation via copy constructor
  Visualization & operator = (const Visualization &); //Prohibit instanciation by copying

  ros::Publisher triangle_pub;
  std::vector<tri_t*> trisToBeVisualized;
public:
  ~Visualization();

  static Visualization& instance();
  void push_back(tri_t* tri);
  static void nextVisualization();
};

class FacetVisualization: public Visualization
{ 
public:
  void visualizeTriangles(); 
  void visualizeTriangles(std::vector<tri_t*> tri);
  friend FacetVisualization& Singleton<FacetVisualization>();
};

class CameraVisualization: public Visualization
{
public:
  void visualizeCameras(StateVector *view_point);
  friend CameraVisualization& Singleton<CameraVisualization>();
};

class PointVisualization: public Visualization
{
public:
  void visualizePoints(std::vector<CartesianCoordinates*> &points);
  friend PointVisualization &Singleton<PointVisualization>();
};

#endif