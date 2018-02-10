#include <gtest/gtest.h>
#include <ros/console.h>
#include "talmech/utility/basic/basic_utility_factory.h"
#include "talmech/utility/basic/distance_utility.h"
#include "talmech/utility/basic/skill_utility.h"

using namespace talmech;
using namespace talmech::utility;
using namespace talmech::utility::basic;

TEST(Utility, Decorator)
{
  RobotPtr robot1(new Robot("robot1"));
  robot1->setUtility("distance skill");
  UtilityComponentPtr component(robot1->getUtilityComponent("DistanceUtility"));
  ROS_WARN_STREAM("[UtilityTest] component " << (component ? component->str() : "-----"));
  DistanceUtilityPtr distance(boost::dynamic_pointer_cast<DistanceUtility>(component));
  if (distance)
  {
    ROS_INFO("Initializing the DistanceUtility component...");
    distance->init(*robot1, 0.5);
  }
  component = robot1->getUtilityComponent("SkillUtility");
  ROS_WARN_STREAM("[UtilityTest] component " << (component ? component->str() : "-----"));
  SkillUtilityPtr skill(boost::dynamic_pointer_cast<SkillUtility>(component));
  if (skill)
  {
    ROS_INFO("Initializing the SkillUtility component...");
    skill->init(*robot1, std::list<double>());
  }
  Waypoint waypoint;
  TaskPtr task1(new Task("task1"));
  task1->addWaypoint(waypoint);
  waypoint.pose.position.x = 3.0;
  task1->addWaypoint(waypoint);
  waypoint.pose.position.x = 0.0;
  waypoint.pose.position.y = 4.0;
  task1->addWaypoint(waypoint);
  waypoint.pose.position.y = 0.0;
  task1->addWaypoint(waypoint);
  double utility(robot1->getUtility(*task1));
  ROS_INFO_STREAM("Utility of " << *robot1 << " for " << *task1 << ": " << utility);
}
