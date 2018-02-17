#include "talmech/exception.h"
#include "talmech/task.h"
#include "talmech/continuous_skill.h"
#include "talmech/discrete_skill.h"

namespace talmech
{
Task::Task(const std::string& id, const Path& waypoints, const Skills& skills)
    : id_(id), waypoints_(new Path(waypoints)), skills_(skills)
{
  if (id_.empty())
  {
    throw Exception("The task id must not be empty.");
  }
}

Task::Task(const Task& task)
    : id_(task.id_), waypoints_(new nav_msgs::Path(*task.waypoints_)),
      skills_(task.skills_)
{
}

Task::Task(const talmech_msgs::Task& msg)
    : id_(msg.id), waypoints_(new nav_msgs::Path(msg.waypoints))
{
  if (id_.empty())
  {
    throw Exception("The task id must not be empty.");
  }
  for (std::size_t i(0); i < msg.skills.size(); i++)
  {
    talmech_msgs::Skill skill_msg(msg.skills[i]);
    SkillPtr skill;
    if (skill_msg.type == 0)
    {
      skill.reset(new Skill(skill_msg));
    }
    else if (skill_msg.type == 1)
    {
      skill.reset(new DiscreteSkill(skill_msg));
    }
    else if (skill_msg.type == 2)
    {
      skill.reset(new ContinuousSkill(skill_msg));
    }
    else
    {
      throw Exception("Invalid Skill ROS message type.");
    }
    skills_.push_back(skill);
  }
}
}
