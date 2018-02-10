#include <gtest/gtest.h>
#include <ros/console.h>
#include "talmech/utility/basic/basic_utility_factory.h"
#include "talmech/utility/basic/distance_utility.h"
#include "talmech/utility/basic/skill_utility.h"
#include "talmech/continuous_skill.h"
#include "talmech/discrete_skill.h"

using namespace talmech;
using namespace talmech::utility;
using namespace talmech::utility::basic;

TEST(Utility, Decorator)
{
  ResourcePtr resource1(new Resource("resource1"));
  ResourcePtr resource2(new Resource("resource2"));
  ResourcePtr resource3(new Resource("resource3"));
  ResourcePtr resource4(new Resource("resource4"));
  ResourcePtr resource5(new Resource("resource5"));
  std::list<double> correction_factors;
  RobotPtr robot1(new Robot("robot1"));
  robot1->addSkill(SkillPtr(new Skill(resource1)));
  correction_factors.push_back(3.6);
  robot1->addSkill(SkillPtr(new ContinuousSkill(resource2, 5.1)));
  correction_factors.push_back(2.4);
  robot1->addSkill(SkillPtr(new DiscreteSkill(resource3, 2)));
  correction_factors.push_back(0.78);
  robot1->addSkill(SkillPtr(new Skill(resource4)));
  correction_factors.push_back(4.7);
  robot1->addSkill(SkillPtr(new DiscreteSkill(resource5, 3)));
  correction_factors.push_back(1.0);
  robot1->setUtility("distance skill");
  UtilityComponentPtr component(robot1->getUtilityComponent("DistanceUtility"));
  DistanceUtilityPtr distance(boost::dynamic_pointer_cast<DistanceUtility>(component));
  if (distance)
  {
    ROS_INFO("Initializing the DistanceUtility component...");
    distance->init(*robot1, 0.5);
  }
  component = robot1->getUtilityComponent("SkillUtility");
  SkillUtilityPtr skill(boost::dynamic_pointer_cast<SkillUtility>(component));
  if (skill)
  {
    ROS_INFO("Initializing the SkillUtility component...");
    skill->init(*robot1, correction_factors);
  }
  Waypoint waypoint;
  TaskPtr task1(new Task("task1"));
  task1->addSkill(SkillPtr(new Skill(resource4)));
  task1->addSkill(SkillPtr(new DiscreteSkill(resource3, 6)));
  task1->addSkill(SkillPtr(new ContinuousSkill(resource2, 2.4)));
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
