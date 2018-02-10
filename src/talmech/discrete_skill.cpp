#include "talmech/discrete_skill.h"
#include <sstream>

namespace talmech
{
std::string DiscreteSkill::str() const
{
  std::stringstream ss;
  ss << Skill::str() << " (" << level_ << ")";
  return ss.str();
}
}
