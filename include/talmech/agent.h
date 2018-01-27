#ifndef _TALMECH_AGENT_H_
#define _TALMECH_AGENT_H_

#include <string>
#include "role.h"

namespace talmech
{
class Agent
{
public:
  typedef boost::shared_ptr<Agent> Ptr;
  typedef boost::shared_ptr<const Agent> ConstPtr;
  virtual ~Agent() {}
  std::string getId() const { return id_; }
  std::string str() const { return id_; }
  const char* c_str() const { return str().c_str(); }
  bool operator==(const Agent& agent) const { return id_ == agent.id_; }
  bool operator!=(const Agent& agent) const { return !(*this == agent); }
protected:
  Agent(const std::string& id, const RolePtr& role = RolePtr());
private:
  std::string id_;
  RolePtr role_;
};
typedef Agent::Ptr AgentPtr;
typedef Agent::ConstPtr AgentConstPtr;
}

#endif // _TALMECH_AGENT_H_
