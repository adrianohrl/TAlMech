#include "talmech/agent.h"
#include "talmech/exception.h"

namespace talmech
{
Agent::Agent(const std::string &id, const utility::UtilityPtr &utility, const RolePtr &role)
  : id_(id), utility_(utility), role_(role), skills_(SkillsPtr(new Skills()))
{
  if (id_.empty())
  {
    throw Exception("The agent's id must not be empty.");
  }
}
}
