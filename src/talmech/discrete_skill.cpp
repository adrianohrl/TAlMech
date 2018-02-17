#include "talmech/discrete_skill.h"
#include <sstream>
#include "talmech/exception.h"

namespace talmech
{
DiscreteSkill::DiscreteSkill(const talmech_msgs::Skill& msg)
    : Skill::Skill(msg), level_(round(msg.level))
{
  if (msg.type != 1)
  {
    throw Exception("This constructor must be used for Skill ROS messages in "
                    "which its type is 1.");
  }
}

std::string DiscreteSkill::str() const
{
  std::stringstream ss;
  ss << Skill::str() << " (" << level_ << ")";
  return ss.str();
}
}
