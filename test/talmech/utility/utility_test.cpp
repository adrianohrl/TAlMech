#include <gtest/gtest.h>
#include <ros/console.h>
#include "talmech/utility/basic/basic_utility_factory.h"
#include "talmech/utility/basic/distance_utility.h"

using namespace talmech;
using namespace talmech::utility;
using namespace talmech::utility::basic;

TEST(Utility, Decorator)
{
  RobotPtr robot1(new Robot("robot1"));
  robot1->setUtility("distance");
  UtilityComponentPtr component(robot1->getUtilityComponent("DistanceUtility"));
  DistanceUtilityPtr distance(boost::dynamic_pointer_cast<DistanceUtility>(component));
  if (distance)
  {
    ROS_INFO("Initializing the DistanceUtility component...");
    distance->init(*robot1, 0.5);
  }
  nav_msgs::Path waypoints;
  geometry_msgs::Pose waypoint;
  waypoints.poses.push_back(waypoint);
  waypoint.position.x = 3.0;
  waypoints.poses.push_back(waypoint);
  waypoint.position.x = 0.0;
  waypoint.position.y = 4.0;
  waypoints.poses.push_back(waypoint);
  waypoint.position.y = 0.0;
  waypoints.poses.push_back(waypoint);
  TaskPtr task1("task1", waypoints);
  ROS_INFO_STREAM("Utility of " << )
}
