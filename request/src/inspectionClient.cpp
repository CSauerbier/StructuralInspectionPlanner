/*!
 * \file inspectionClient.cpp
 *
 * More elaborate description
 */
#include <ros/ros.h>
#include "koptplanner/inspection.h"
#include "shape_msgs/SolidPrimitive.h"
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <ros/package.h>
#include "tf/tf.h"
#include <math.h>

#include "request/MeshResolutionModification.h"

std::vector<nav_msgs::Path> * readSTLfile(std::string name);
std::vector<nav_msgs::Path> * readSTLfile(std::string name, double *max_x, double *max_y, double *max_z, double *min_x, double *min_y, double *min_z);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inspectionClient");
  if (argc != 1)
  {
    ROS_INFO("usage: plan");
    return 1;
  }

  ros::NodeHandle n;
  ros::Publisher obstacle_pub = n.advertise<visualization_msgs::Marker>("scenario", 1);
  ros::ServiceClient client = n.serviceClient<koptplanner::inspection>("inspectionPath");
  
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_publisher", 1);

  std::string coarse_file_path, fine_file_path;
  request::MeshResolutionModification mesh_srv;
  std::string file_path_in, gripper_file_path;
  float target_resolution_coarse, target_resolution_fine;

  ros::Rate r(50.0);
  ros::Rate r2(1.0);
  r2.sleep();

  double max_x, max_y, max_z, min_x, min_y, min_z; //TO-DO: Move down, consider array

  /* define the bounding box */
  koptplanner::inspection srv;
  srv.request.spaceSize.push_back(2000);
  srv.request.spaceSize.push_back(2000);
  srv.request.spaceSize.push_back(2000);
  // srv.request.spaceCenter.push_back(0);
  // srv.request.spaceCenter.push_back(0);
  // srv.request.spaceCenter.push_back(0);
  geometry_msgs::Pose reqPose;

  /* starting pose*/
  reqPose.position.x = 25.0;
  reqPose.position.y = 25.0;
  reqPose.position.z = -55.0;
  tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
  reqPose.orientation.x = q.x();
  reqPose.orientation.y = q.y();
  reqPose.orientation.z = q.z();
  reqPose.orientation.w = q.w();
  srv.request.requiredPoses.push_back(reqPose);

  /* final pose (remove if no explicit final pose is desired) */
  reqPose.position.x = 25.0;
  reqPose.position.y = 25.0;
  reqPose.position.z = -55;
  q = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
  reqPose.orientation.x = q.x();
  reqPose.orientation.y = q.y();
  reqPose.orientation.z = q.z();
  reqPose.orientation.w = q.w();
  srv.request.requiredPoses.push_back(reqPose);

  /* parameters for the path calculation (such as may change during mission) */
  //TO-DO: Add handling for missing parameters (ros::param::has("my_param")) and consider parametrizing more variables
  ros::param::get("~/camera/incidence_angle",srv.request.incidenceAngle);
  srv.request.incidenceAngle = srv.request.incidenceAngle*M_PI/180;

  ros::param::get("~/camera/min_dist", srv.request.minDist);
  ros::param::get("~/camera/max_dist", srv.request.maxDist);
  srv.request.numIterations = 1;
  //TO-DO: Remove iterations


  ros::ServiceClient service_call = n.serviceClient<request::MeshResolutionModification>("/mesh_resolution_modification");

  ros::param::get("~/mesh/path", file_path_in);
  ros::param::get("~/mesh/target_resolution_coarse", target_resolution_coarse);
  ros::param::get("~/mesh/target_resolution_fine", target_resolution_fine);

  mesh_srv.request.file_path_in = file_path_in;
  mesh_srv.request.target_resolution = target_resolution_coarse;
  service_call.waitForExistence();
  if(!service_call.call(mesh_srv))
  {
    ROS_ERROR("Mesh-Service call failed");
    return 1;
  }
  coarse_file_path = mesh_srv.response.file_path_out;

  mesh_srv.request.target_resolution = target_resolution_fine;
  service_call.waitForExistence();
  if(!service_call.call(mesh_srv)) 
  {
    ROS_ERROR("Mesh-Service call failed");
    return 1;
  }
  fine_file_path = mesh_srv.response.file_path_out;


  /* read STL file and publish to rviz */
  std::vector<nav_msgs::Path> * mesh = readSTLfile(fine_file_path, &max_x, &max_y, &max_z, &min_x, &min_y, &min_z);
  srv.request.spaceCenter.push_back((max_x+min_x)/2);
  srv.request.spaceCenter.push_back((max_y+min_y)/2);
  srv.request.spaceCenter.push_back((max_z+min_z)/2);
  ROS_INFO("mesh size = %i", (int)mesh->size());
  for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++)
  {
    geometry_msgs::Polygon p;
    geometry_msgs::Point32 p32;
    p32.x = it->poses[0].pose.position.x;
    p32.y = it->poses[0].pose.position.y;
    p32.z = it->poses[0].pose.position.z;
    p.points.push_back(p32);
    p32.x = it->poses[1].pose.position.x;
    p32.y = it->poses[1].pose.position.y;
    p32.z = it->poses[1].pose.position.z;
    p.points.push_back(p32);
    p32.x = it->poses[2].pose.position.x;
    p32.y = it->poses[2].pose.position.y;
    p32.z = it->poses[2].pose.position.z;
    p.points.push_back(p32);
    srv.request.inspectionArea.push_back(p);
  }

  /* read in simplified coarse mesh for preprocessing*/
  std::vector<nav_msgs::Path> *mesh2 = readSTLfile(coarse_file_path);
  if(mesh2 != NULL)
  {
    ROS_INFO("mesh size = %i", (int)mesh2->size());
    for(std::vector<nav_msgs::Path>::iterator it = mesh2->begin(); it != mesh2->end() && ros::ok(); it++)
    {
      geometry_msgs::Polygon p;
      geometry_msgs::Point32 p32;
      p32.x = it->poses[0].pose.position.x;
      p32.y = it->poses[0].pose.position.y;
      p32.z = it->poses[0].pose.position.z;
      p.points.push_back(p32);
      p32.x = it->poses[1].pose.position.x;
      p32.y = it->poses[1].pose.position.y;
      p32.z = it->poses[1].pose.position.z;
      p.points.push_back(p32);
      p32.x = it->poses[2].pose.position.x;
      p32.y = it->poses[2].pose.position.y;
      p32.z = it->poses[2].pose.position.z;
      p.points.push_back(p32);
      srv.request.inspectionAreaCoarse.push_back(p);
    }
  }

  /* read in gripper geometry if given*/
  ros::param::get("~/mesh/gripper_path", gripper_file_path);
  if(gripper_file_path != "")
  {
    std::vector<nav_msgs::Path> *mesh_gripper = readSTLfile(gripper_file_path);

    for(std::vector<nav_msgs::Path>::iterator it = mesh_gripper->begin(); it != mesh_gripper->end() && ros::ok(); it++)
    {
      geometry_msgs::Polygon p;
      geometry_msgs::Point32 p32;
      p32.x = it->poses[0].pose.position.x;
      p32.y = it->poses[0].pose.position.y;
      p32.z = it->poses[0].pose.position.z;
      p.points.push_back(p32);
      p32.x = it->poses[1].pose.position.x;
      p32.y = it->poses[1].pose.position.y;
      p32.z = it->poses[1].pose.position.z;
      p.points.push_back(p32);
      p32.x = it->poses[2].pose.position.x;
      p32.y = it->poses[2].pose.position.y;
      p32.z = it->poses[2].pose.position.z;
      p.points.push_back(p32);
      srv.request.gripper.push_back(p);
    }
  }

  if (client.call(srv))
  {
    /* Publish the inspection path for the robot */
    path_pub.publish(srv.response.inspectionPath);

    /* writing results of scenario to m-file. use supplied script to visualize */
    std::fstream pathPublication;
    std::string pkgPath = ros::package::getPath("request");
    pathPublication.open((pkgPath+"/visualization/inspectionScenario.m").c_str(), std::ios::out);
    if(!pathPublication.is_open())
    {
      ROS_ERROR("Could not open 'inspectionScenario.m'! Inspection path is not written to file");
      return 1;
    }
    pathPublication << "inspectionPath = [";
    for(std::vector<geometry_msgs::PoseStamped>::iterator it = srv.response.inspectionPath.poses.begin(); it != srv.response.inspectionPath.poses.end(); it++)
    {
      tf::Pose pose;
      tf::poseMsgToTF(it->pose, pose);
      double yaw_angle = tf::getYaw(pose.getRotation());
      pathPublication << it->pose.position.x << ", " << it->pose.position.y << ", " << it->pose.position.z << ", 0, 0, " << yaw_angle << ";\n";
    }
    pathPublication << "];\n";
    pathPublication << "inspectionCost = " << srv.response.cost << ";\n";
    pathPublication << "numObstacles = " << std::min(srv.request.obstacles.size(),srv.request.obstaclesPoses.size()) << ";\n";
    for(int i = 0; i<std::min(srv.request.obstacles.size(),srv.request.obstaclesPoses.size()); i++)
    {
      pathPublication << "obstacle{" << i+1 << "} = [" << srv.request.obstacles[i].dimensions[0] << ", " << srv.request.obstacles[i].dimensions[1] << ", " << srv.request.obstacles[i].dimensions[2] << ";\n";
      pathPublication << srv.request.obstaclesPoses[i].position.x << ", " << srv.request.obstaclesPoses[i].position.y << ", " << srv.request.obstaclesPoses[i].position.z << "];\n";
    }
    pathPublication << "meshX = [";
    for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++)
    {
      pathPublication << it->poses[0].pose.position.x << ", " << it->poses[1].pose.position.x << ", " << it->poses[2].pose.position.x << ";\n";
    }
    pathPublication << "];\nmeshY = [";
    for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++)
    {
      pathPublication << it->poses[0].pose.position.y << ", " << it->poses[1].pose.position.y << ", " << it->poses[2].pose.position.y << ";\n";
    }
    pathPublication << "];\nmeshZ = [";
    for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++)
    {
      pathPublication << it->poses[0].pose.position.z << ", " << it->poses[1].pose.position.z << ", " << it->poses[2].pose.position.z << ";\n";
    }
    pathPublication << "];\n";
  }
  else
  {
    ROS_ERROR("Failed to call service planner");
    return 1;
  }

  return 0;
}

/**
*  \brief This function reads an ACII STL file for inspection planning
*/
std::vector<nav_msgs::Path> * readSTLfile(std::string name)
{
  return readSTLfile(name, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
}

/**
*  \brief This function reads an ACII STL file for inspection planning. Parameters other than name are filled with outmost mesh mesh coordinates
*/
std::vector<nav_msgs::Path> * readSTLfile(std::string name, double *max_x, double *max_y, double *max_z, double *min_x, double *min_y, double *min_z)
{
  std::vector<nav_msgs::Path> * mesh = new std::vector<nav_msgs::Path>;
  std::fstream f;
  f.open(name.c_str());
  // assert(f.is_open());
  if (!f.is_open()) return NULL;
  int MaxLine = 0;
  char* line;
  double maxX = -DBL_MAX;
  double maxY = -DBL_MAX;
  double maxZ = -DBL_MAX;
  double minX = DBL_MAX;
  double minY = DBL_MAX;
  double minZ = DBL_MAX;
  assert(line = (char *) malloc(MaxLine = 80));
  f.getline(line, MaxLine);
  if(0 != strcmp(strtok(line, " "), "solid"))
  {
    ROS_ERROR("Invalid mesh file! Make sure the file is given in ascii-format.");
    ros::shutdown();
  }
  assert(line = (char *) realloc(line, MaxLine));
  f.getline(line, MaxLine);
  int k = 0;
  while(0 != strcmp(strtok(line, " "), "endsolid") && !ros::isShuttingDown())
  {
    int q = 0;
    nav_msgs::Path p;
    geometry_msgs::PoseStamped v1;
    for(int i = 0; i<7; i++)
    {
      while(line[q] == ' ')
        q++;
      if(line[q] == 'v')
      {
        const double yawTrafo = 0.0;      // used to rotate the mesh before processing
        const double scaleFactor = 1.0;   // used to scale the mesh before processing
        const double offsetX = 0.0;       // used to offset the mesh before processing
        const double offsetY = 0.0;       // used to offset the mesh before processing
        const double offsetZ = 0.0;       // used to offset the mesh before processing

        geometry_msgs::PoseStamped vert;
        char* v = strtok(line+q," ");
        v = strtok(NULL," ");
        double xtmp = atof(v)/scaleFactor;
        v = strtok(NULL," ");
        double ytmp = atof(v)/scaleFactor;
        vert.pose.position.x = cos(yawTrafo)*xtmp-sin(yawTrafo)*ytmp;
        vert.pose.position.y =  sin(yawTrafo)*xtmp+cos(yawTrafo)*ytmp;
        v = strtok(NULL," ");
        vert.pose.position.z =  atof(v)/scaleFactor;
        vert.pose.position.x -= offsetX;
        vert.pose.position.y -= offsetY;
        vert.pose.position.z -= offsetZ;
        if(maxX<vert.pose.position.x)
          maxX=vert.pose.position.x;
        if(maxY<vert.pose.position.y)
          maxY=vert.pose.position.y;
        if(maxZ<vert.pose.position.z)
          maxZ=vert.pose.position.z;
        if(minX>vert.pose.position.x)
          minX=vert.pose.position.x;
        if(minY>vert.pose.position.y)
          minY=vert.pose.position.y;
        if(minZ>vert.pose.position.z)
          minZ=vert.pose.position.z;
        vert.pose.orientation.x =  0.0;
        vert.pose.orientation.y =  0.0;
        vert.pose.orientation.z =  0.0;
        vert.pose.orientation.w =  1.0;
        p.poses.push_back(vert);
        if(p.poses.size() == 1)
          v1 = vert;
      }
      assert(line = (char *) realloc(line, MaxLine));
      f.getline(line, MaxLine);
    }
    p.poses.push_back(v1);
    p.header.frame_id = "kopt_frame";
    p.header.stamp = ros::Time::now();
    p.header.seq = k;
    mesh->push_back(p);
    k++;
  }
  if(max_x != nullptr && max_y != nullptr && max_z != nullptr && min_x != nullptr && min_y != nullptr && min_z != nullptr)
  {
    //TO-DO: Consider using array
    *max_x = maxX;
    *max_y = maxY;
    *max_z = maxZ;
    *min_x = minX;
    *min_y = minY;
    *min_z = minZ;
  }
  free(line);
  f.close();
  ROS_INFO("Mesh area is bounded by: [%2.2f,%2.2f]x[%2.2f,%2.2f]x[%2.2f,%2.2f]", minX,maxX,minY,maxY,minZ,maxZ);
  return mesh;
}
