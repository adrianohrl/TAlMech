#include "talmech/continuous_skill.h"
#include <sstream>
#include "talmech/exception.h"

namespace talmech
{
ContinuousSkill::ContinuousSkill(const talmech_msgs::Skill& msg)
    : Skill::Skill(msg), level_(msg.level)
{
  if (msg.type != 2)
  {
    throw Exception("This constructor must be used for Skill ROS messages in "
                    "which its type is 2.");
  }
}

std::string ContinuousSkill::str() const
{
  std::stringstream ss;
  ss << Skill::str() << " (" << level_ << ")";
  return ss.str();
}
}
