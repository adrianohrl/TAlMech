#include "talmech/continuous_skill.h"
#include <sstream>

namespace talmech
{
std::string ContinuousSkill::str() const
{
  std::stringstream ss;
  ss << Skill::str() << "(" << level_ << ")";
  return ss.str();
}
}
