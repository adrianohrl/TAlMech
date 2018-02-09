#include "talmech/agent.h"
#include "talmech/exception.h"

namespace talmech
{
Agent::Agent(const std::string &id, const utility::UtilityFactoryPtr &factory, const RolePtr &role)
  : id_(id), factory_(factory), role_(role)
{
  if (id_.empty())
  {
    throw Exception("The agent's id must not be empty.");
  }
}
}
