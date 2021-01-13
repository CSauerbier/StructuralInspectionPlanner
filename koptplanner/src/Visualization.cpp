

#include "Visualization/Visualization.h"
#include "Visualization/Colors.h"
#include "tf/tf.h"

int Visualization::s_publishing_number = 0;

Visualization::Visualization()
{
  //s_publishing_number = 0;
}

Visualization::~Visualization()
{
}

Visualization& Visualization::instance()
{
  static Visualization _instance;
  return _instance;
}

void Visualization::push_back(tri_t* tri)
{
  this->trisToBeVisualized.push_back(tri);
}

void Visualization::nextVisualization()
{
  s_publishing_number++;
  ColorRGB::cycle();
}

void FacetVisualization::visualizeTriangles()
{
  this->visualizeTriangles(this->trisToBeVisualized);
  this->trisToBeVisualized.clear();
}

void FacetVisualization::visualizeTriangles(std::vector<tri_t*> tri)
{
  ros::NodeHandle n;
  
  ros::Publisher marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("triangle_visualization", 10);

  ros::Rate r(30);

  for (int iter=0; iter<10; iter++) //TO-DO: Improve. Redundancy, because not every message is received by subscriber
  {

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "kopt_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "triangle_list";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.z = 1;
    marker.color.r = ColorRGB::getCurrent().r;
    marker.color.g = ColorRGB::getCurrent().g;
    marker.color.b = ColorRGB::getCurrent().b;
    marker.color.a = 1.0;

    for (uint32_t i = 0; i < tri.size(); ++i)
    {
      tri_t* temp_tri = tri.at(i);

      geometry_msgs::Point p;
      p.x = temp_tri->x1.coeff(0);
      p.y = temp_tri->x1.coeff(1);
      p.z = temp_tri->x1.coeff(2);
      
      marker.points.push_back(p);
      p.x = temp_tri->x2.coeff(0);
      p.y = temp_tri->x2.coeff(1);
      p.z = temp_tri->x2.coeff(2);
      marker.points.push_back(p);
      p.x = temp_tri->x3.coeff(0);
      p.y = temp_tri->x3.coeff(1);
      p.z = temp_tri->x3.coeff(2);
      marker.points.push_back(p);


      marker.id = i + (s_publishing_number)*10000; //TO-DO: Improve. Create an unique ID for each facet

      marker_array.markers.push_back(marker);
      marker.points.clear();

      p.z += 1.0;
    }

    marker_array_pub.publish(marker_array);

    r.sleep();
  }
}

void CameraVisualization::visualizeCameras(StateVector *view_point)
{
  ros::NodeHandle n;
  ros::Publisher viewpoint_pub = n.advertise<visualization_msgs::Marker>("viewpoint_marker", 1);

  /* display sampled viewpoint in rviz */
  visualization_msgs::Marker point;
  point.header.frame_id = "kopt_frame"; //TO-DO: Check backward compatibility to lower ROS distros than noetic
  point.header.stamp = ros::Time::now();
  point.id = s_publishing_number;
  point.ns = "Viewpoints";
  point.type = visualization_msgs::Marker::ARROW;
  point.pose.position.x = view_point->coeff(0);
  point.pose.position.y = view_point->coeff(1);
  point.pose.position.z = view_point->coeff(2);

  #if DIMENSIONALITY>4
  tf::Quaternion q = tf::createQuaternionFromRPY(VP[i][3],VP[i][4],VP[i][5]);
  #else
  tf::Quaternion q = tf::createQuaternionFromRPY(0,view_point->coeff(4),view_point->coeff(3));
  #endif
  point.pose.orientation.x = q.x();
  point.pose.orientation.y = q.y();
  point.pose.orientation.z = q.z();
  point.pose.orientation.w = q.w();

  //sqrt(SQ(problemBoundary.size[0])+SQ(problemBoundary.size[1])+SQ(problemBoundary.size[2]))/70.0; //TO-DO: Introduce parameter
  double scaleVP = 30.0;
  point.scale.x = 2*scaleVP;
  point.scale.y = scaleVP/10.0;
  point.scale.z = scaleVP/10.0;
  point.color.r = ColorRGB::getCurrent().r;
  point.color.g = ColorRGB::getCurrent().g;
  point.color.b = ColorRGB::getCurrent().b;
  point.color.a = 0.7;
  viewpoint_pub.publish(point);
}


void PointVisualization::visualizePoints(std::vector<CartesianCoordinates*> &points)
{
  ros::NodeHandle n;
  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("point_visualization", 10);

  ros::Rate r(30);
  r.sleep();

  for (int iter=0; iter<10; iter++) //TO-DO: Improve. Redundancy, because not every message is received by subscriber
  {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "kopt_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points_list";
    // marker.action = visualization_msgs::Marker::ADD;  //TO-DO: See if this can be used to handle overwriting
    marker.pose.orientation.w = 1.0;

    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = marker.scale.y = marker.scale.z = 1;
    marker.color.r = ColorRGB::getCurrent().r;
    marker.color.g = ColorRGB::getCurrent().g;
    marker.color.b = ColorRGB::getCurrent().b;
    marker.color.a = 1.0;

    int i=0;
    for (auto point: points)
    {
      geometry_msgs::Point p;
      p.x = point->x;
      p.y = point->y;
      p.z = point->z;

      marker.points.push_back(p);

      marker.id = i + s_publishing_number+1000;  //TO-DO: Fix this for rest of code
      i++;
    }

    marker_pub.publish(marker);

    r.sleep();
  }
}